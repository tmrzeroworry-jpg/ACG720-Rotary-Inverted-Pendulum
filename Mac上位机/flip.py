"""One explicit revolution request; FPGA performs trajectory/energy/capture locally."""

import secrets, time
from trajectory import packet
from protocol_vendor import crc16

PHASE = {
    0: "未启动",
    1: "平滑离开直立",
    2: "同向翻转",
    3: "顶部捕获",
    4: "确认稳摆",
    5: "翻转完成",
    6: "翻转未完成",
}
REASON = {
    0: "",
    1: "启动条件不满足，请先停止横臂任务并稳定直立",
    2: "动作超时",
    3: "检测到主行程反向",
    4: "电位器角度出现不可信跳变",
    5: "横臂速度/行程或摆杆速度超限",
    6: "错过顶部捕获窗口",
    7: "捕获或稳摆确认失败",
    8: "动作已中止",
    9: "传感器失效",
    10: "控制器保护或停止",
    12: "电位器端点跨区超时",
    13: "端点以外角度异常或跨区未确认",
}


def flip_packet(op, seq, session, direction=1, variant=0):
    b = bytearray(packet(op, seq, session))
    b[12:16] = int(direction * (variant + 1) if op == 4 else 0).to_bytes(
        4, "big", signed=True
    )
    b[20:22] = crc16(b[:20]).to_bytes(2, "big")
    return bytes(b)


class Flip:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.phase = "idle"
        self.owner = None
        self.session = 0
        self.seq = 0
        self.old_id = 0
        self.direction = 1
        self.variant = 0
        self.sent = -1.0
        self.deadline = 0.0
        self.heartbeat = 0.0
        self.issued = False
        self.start_seq = None
        self.abort_sent = False
        self.message = "稳摆后可执行一次同向翻转。"
        self.events = []

    def event(self, name, **kw):
        self.events.append(dict(at=self.clock(), event=name, **kw))

    def disconnect(self):
        self.phase = "idle"
        self.message = "未连接，不自动恢复翻转。"

    def start(self, e, client, direction, variant=0):
        if self.phase in ("hello", "pending", "running", "aborting"):
            raise ValueError("翻转任务仍在进行。")
        if 1 <= (e.latest or {}).get("life_phase", 0) <= 8:
            raise ValueError("请先等待起摆/回落任务结束。")
        if e.trajectory.phase in ("hello", "arming", "running", "stopping"):
            raise ValueError("请先减速停止自绘曲线。")
        if not isinstance(client, str) or not 8 <= len(client) <= 80:
            raise ValueError("无效页面标识。")
        if type(direction) is not int or direction not in (-1, 1):
            raise ValueError("方向只能是+1或-1。")
        if type(variant) is not int or variant not in (0, 1):
            raise ValueError("方案只能选A或B。")
        f = e.latest
        if (
            e.mode != "live"
            or not f
            or self.clock() - e.last_receive > 0.3
            or (f["build"], f.get("protocol_version")) != ("UI02", 8)
        ):
            raise ValueError("需要新鲜的UI02 V8实机数据。")
        if f["state"] != 4:
            raise ValueError("先点击起摆，稳摆后执行翻转。")
        if f.get("stream_velocity_q16") != 0:
            raise ValueError("先等待横臂速度参考降到0。")
        self.owner = client
        self.direction = direction
        self.variant = variant
        self.session = secrets.randbits(32) or 1
        self.seq = 0
        self.old_id = f["flip_id"]
        self.phase = "hello"
        self.sent = -1
        self.deadline = self.clock() + 1.5
        self.heartbeat = self.clock()
        self.issued = False
        self.start_seq = None
        self.abort_sent = False
        self.message = "正在请求翻转控制权…"
        self.event(
            "request", direction=direction, variant=variant, session=self.session
        )

    def pulse(self, client):
        if client != self.owner:
            raise ValueError("此页面没有当前翻转的心跳权限。")
        self.heartbeat = self.clock()

    def cancel(self):
        if self.phase == "hello":
            self.phase = "error"
            self.message = "已取消，未发送翻转启动。"
        elif self.phase in ("pending", "running"):
            self.phase = "aborting"
            self.abort_sent = False
            self.deadline = self.clock() + 1
            self.message = "已请求中止翻转。"

    def step(self, e):
        now = self.clock()
        f = e.latest
        if self.phase in ("idle", "done", "error"):
            return None
        if (
            e.mode != "live"
            or not f
            or now - e.last_receive > 0.3
            or f["build"] != "UI02"
        ):
            self.phase = "error"
            self.message = "遥测中断，停止发送；板端看门狗会中止动作。"
            return None
        if now - self.heartbeat > 0.5:
            self.cancel()
        if self.phase == "error":
            return None
        same = f.get("stream_session") == self.session
        if self.phase == "hello":
            if now > self.deadline:
                self.phase = "error"
                self.message = "握手未确认，未启动翻转。"
                return None
            if same and f.get("stream_flags", 0) & 32:
                self.phase = "pending"
                self.deadline = now + 1
                self.sent = -1
        elif self.phase == "pending":
            if f.get("flip_id") != self.old_id and f.get("flip_phase") in (1, 2, 3, 4):
                self.phase = "running"
                self.sent = -1
                self.deadline = now + 7
            elif (
                self.issued
                and same
                and f.get("stream_seq") == self.start_seq
                and f.get("flip_result")
            ):
                self.phase = "error"
                self.message = REASON.get(f["flip_result"], "板端拒绝启动")
                return None
            elif now > self.deadline:
                self.phase = "error"
                self.message = "启动未确认，不自动重发启动命令。"
                return None
        if self.phase in ("running", "aborting"):
            if f.get("flip_id") != self.old_id and f.get("flip_phase") == 5:
                self.phase = "done"
                self.message = "已完成同向一整圈并重新确认稳摆。"
                self.event("completed", progress=f.get("flip_progress"))
                return None
            if f.get("flip_phase") == 6:
                self.phase = "error"
                self.message = REASON.get(f.get("flip_result"), "翻转未完成")
                self.event("failed", reason=f.get("flip_result"))
                return None
            if now > self.deadline:
                self.cancel()
        if self.phase == "aborting":
            if self.abort_sent:
                if now > self.deadline:
                    self.phase = "error"
                    self.message = "等待中止确认超时，板端看门狗仍生效。"
                return None
            op = 6
            self.abort_sent = True
        elif self.phase == "hello":
            if now - self.sent < 0.1:
                return None
            op = 0
        elif self.phase == "pending":
            if self.issued:
                return None
            op = 4
            self.issued = True
        elif self.phase == "running":
            if now - self.sent < 0.1:
                return None
            op = 5
        else:
            return None
        self.seq = (self.seq + 1) & 65535
        self.sent = now
        if op == 4:
            self.start_seq = self.seq
        b = flip_packet(op, self.seq, self.session, self.direction, self.variant)
        self.event("tx", op=op, seq=self.seq, session=self.session, hex=b.hex())
        return b

    def status(self):
        return dict(
            phase=self.phase,
            message=self.message,
            direction=self.direction,
            variant=self.variant,
            session=self.session,
        )
