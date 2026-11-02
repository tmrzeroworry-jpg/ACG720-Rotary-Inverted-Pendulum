"""UI02 high-level control. Explicit claim, one outstanding command, no reconnect resume."""

import secrets, struct, time
from protocol_vendor import crc16

OPS = {"start": 2, "position": 3, "spin": 4, "sine": 5, "hold": 6}
MESSAGES = {
    0: "板端已确认",
    2: "参数超出范围",
    3: "当前状态不接受此动作",
    4: "板上S0已急停",
    5: "控制心跳中断，板端已停止",
}


def packet(op, seq, session, arg=0):
    b = struct.pack(">4sBBHIIHH", b"\xaa\xc4\x01\x18", op, 0, seq, session, arg, 0, 500)
    return b + struct.pack(">H", crc16(b)) + b"\r\n"


class Controls:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.phase = "idle"
        self.message = "连接UI02后启用网页控制。"
        self.owner = None
        self.session = 0
        self.seq = 0
        self.sent = -1
        self.heartbeat = 0
        self.pending = None
        self.waiting = None
        self.deadline = 0
        self.events = []
        self.issued = False

    def event(self, name, **kw):
        self.events.append(dict(at=self.clock(), event=name, **kw))

    def disconnect(self):
        self.phase = "idle"
        self.owner = None
        self.pending = None
        self.waiting = None
        self.message = "未接管；连接不会自动启动。"

    def fresh(self, e):
        f = e.latest
        if (
            e.mode != "live"
            or not f
            or self.clock() - e.last_receive > 0.3
            or (f.get("build"), f.get("protocol_version")) != ("UI02", 8)
        ):
            raise ValueError("请连接UI02 V8实机；回放和旧固件不发送控制指令。")
        return f

    def claim(self, e, client):
        f = self.fresh(e)
        if not isinstance(client, str) or not 8 <= len(client) <= 80:
            raise ValueError("页面标识无效。")
        if (
            self.phase in ("claiming", "active", "stopping")
            or f.get("remote_flags", 0) & 1
        ):
            raise ValueError("控制已被接管；先停止或释放当前控制。")
        if f.get("remote_flags", 0) & 12 != 12:
            raise ValueError("请先松开板上S0，并等待传感器就绪。")
        if f["state"] not in (1, 4) or self.other_busy(e):
            raise ValueError("请等待当前动作结束。")
        self.owner = client
        self.session = secrets.randbits(32) or 1
        self.seq = 0
        self.sent = -1
        self.phase = "claiming"
        self.heartbeat = self.clock()
        self.issued = False
        self.deadline = self.clock() + 1
        self.pending = None
        self.waiting = None
        self.message = "正在接管，尚未启动运动。"
        self.event("claim", session=self.session)

    def require_owner(self, e, client):
        f = self.fresh(e)
        if (
            self.phase != "active"
            or client != self.owner
            or f.get("remote_session") != self.session
            or not f.get("remote_flags", 0) & 1
        ):
            raise ValueError("请在当前页面启用网页控制。")
        if self.pending or self.waiting:
            raise ValueError("上一条指令尚未确认，请稍候。")
        return f

    def other_busy(self, e):
        f = e.latest or {}
        return (
            1 <= f.get("life_phase", 0) <= 8
            or e.flip.phase in ("hello", "pending", "running", "aborting")
            or e.return_motion.phase
            in ("waiting_curve", "hello", "pending", "running", "aborting")
            or e.trajectory.phase in ("hello", "arming", "running", "stopping")
        )

    def action(self, e, client, action, value=0):
        f = self.require_owner(e, client)
        if f.get("remote_flags", 0) & 12 != 12:
            raise ValueError("板上S0或传感器尚未就绪。")
        if action not in OPS or isinstance(value, bool) or not isinstance(value, int):
            raise ValueError("无效操作或参数。")
        if self.other_busy(e):
            raise ValueError("请先结束曲线、翻转或回落，不能同时执行。")
        if action == "start":
            if value not in (0, 1) or f["state"] != 1:
                raise ValueError("起摆仅在待机时可用。")
            if value == 0 and not f.get("life_down_valid"):
                raise ValueError("先让摆杆自然下垂静置，等待下垂参考就绪。")
        elif f["state"] != 4:
            raise ValueError("请先完成起摆并进入稳摆。")
        if action in ("position", "sine") and not 0 <= value <= 7:
            raise ValueError("选项范围0～7。")
        if action == "spin" and not 0 <= value <= 3:
            raise ValueError("无效方向或速度档。")
        if action == "hold" and value != 0:
            raise ValueError("无效停止参数。")
        self.pending = (OPS[action], value)
        self.message = "正在发送操作…"
        self.event("request", action=action, value=value)

    def pulse(self, client):
        if client != self.owner or self.phase not in ("claiming", "active"):
            raise ValueError("此页面无控制权限。")
        self.heartbeat = self.clock()

    def stop(self, e):
        self.fresh(e)
        # Stop never waits behind another action or requires the old owner's token.
        e.trajectory.disconnect()
        e.flip.disconnect()
        e.return_motion.disconnect()
        self.pending = None
        self.waiting = None
        self.phase = "stopping"
        self.issued = False
        self.sent = -1
        self.deadline = self.clock() + 0.8
        self.session = secrets.randbits(32) or 1
        self.seq = 0
        self.message = "正在立即停止…"
        self.event("stop")

    def special(self, e, client):
        f = self.require_owner(e, client)
        if f.get("remote_mode", 0) and f.get("remote_cfg") != 128:
            raise ValueError("先点击“停止横臂任务”，再选择其他动作。")

    def tx(self, op, arg=0):
        self.seq = (self.seq + 1) & 65535
        self.sent = self.clock()
        b = packet(op, self.seq, self.session, arg)
        self.event(
            "tx", op=op, arg=arg, seq=self.seq, session=self.session, hex=b.hex()
        )
        return b

    def step(self, e):
        if self.phase not in ("claiming", "active", "stopping"):
            return None
        now = self.clock()
        f = e.latest or {}
        if (
            e.mode != "live"
            or now - e.last_receive > 0.3
            or (f.get("build"), f.get("protocol_version")) != ("UI02", 8)
        ):
            self.phase = "error"
            self.owner = None
            self.message = "通信中断；板端心跳超时停止，不自动恢复。"
            return None
        same = f.get("remote_session") == self.session
        if self.phase == "stopping":
            if (
                self.issued
                and same
                and f.get("remote_seq") == self.seq
                and f.get("remote_op") == 7
            ):
                self.phase = "idle"
                self.owner = None
                self.message = "已停止并释放控制；再次运行需重新启用。"
                return None
            if now > self.deadline:
                self.phase = "error"
                self.owner = None
                self.message = "停止确认超时，请使用板上S0。"
                return None
            if not self.issued:
                self.issued = True
                return self.tx(7)
            return None
        if now - self.heartbeat > 0.6:
            self.stop(e)
            self.message = "网页心跳中断，正在停止。"
            return self.step(e)
        if self.phase == "claiming":
            if same and f.get("remote_flags", 0) & 1:
                self.phase = "active"
                self.message = "网页控制已启用；板上S0始终有效。"
            elif now > self.deadline:
                self.phase = "error"
                self.owner = None
                self.message = "接管未确认，没有启动运动。"
                return None
            elif not self.issued:
                self.issued = True
                return self.tx(0)
            else:
                return None
        if not same or not f.get("remote_flags", 0) & 1:
            self.phase = "error"
            self.owner = None
            self.pending = None
            self.waiting = None
            self.message = MESSAGES.get(
                f.get("remote_result"), "控制已释放，需手动重新启用。"
            )
            return None
        if self.waiting:
            seq, op, deadline = self.waiting
            if f.get("remote_seq") == seq and f.get("remote_op") == op:
                self.message = MESSAGES.get(f.get("remote_result"), "板端拒绝操作")
                self.waiting = None
            elif now > deadline:
                self.stop(e)
                self.message = "指令确认超时，停止后不重发启动。"
                return self.step(e)
            else:
                return None
        if self.pending:
            op, arg = self.pending
            self.pending = None
            b = self.tx(op, arg)
            self.waiting = (self.seq, op, now + 0.25)
            return b
        if now - self.sent >= 0.1:
            return self.tx(1)
        return None

    def status(self):
        return dict(
            phase=self.phase,
            message=self.message,
            session=self.session,
            owner=self.owner,
            pending=bool(self.pending or self.waiting),
        )
