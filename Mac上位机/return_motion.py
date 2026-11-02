"""Explicit controlled return; no automatic starts or arbitrary motor commands."""

import time, secrets
from trajectory import packet

ACTIVE = ("waiting_curve", "hello", "pending", "running", "aborting")
PHASE = {
    0: "待机",
    1: "柔和启动",
    2: "能量起摆",
    3: "捕获与稳摆确认",
    4: "停止横臂任务",
    5: "引导离开直立",
    6: "主动收能下落",
    7: "下垂反馈消摆",
    8: "静止释放确认",
    9: "起摆完成",
    10: "已回到下垂",
    11: "动作未完成",
}
REASON = {
    1: "启动条件或下垂参考尚未就绪",
    2: "动作超时",
    3: "传感器数据过期",
    4: "角度异常/端点跨区失败",
    5: "速度或横臂行程超限",
    6: "下落行程超限",
    7: "起摆换向预算用尽",
    8: "原控制器未完成交接",
    9: "横臂未能在准备时间内停稳",
    10: "原控制器保护/停止",
    11: "动作已中止",
}


class ReturnMotion:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.phase = "idle"
        self.message = "稳摆后可主动回到下垂。"
        self.owner = None
        self.heartbeat = 0
        self.sent = -1
        self.deadline = 0
        self.session = 0
        self.seq = 0
        self.old_id = 0
        self.issued = False
        self.abort_sent = False
        self.events = []
        self.start_seq = None

    def event(self, name, **kw):
        self.events.append(dict(at=self.clock(), event=name, **kw))

    def disconnect(self):
        self.phase = "idle"
        self.message = "未连接；不会自动发起回落。"

    def start(self, e, client):
        f = e.latest
        if self.phase in ACTIVE:
            raise ValueError("回落任务已在进行。")
        if not isinstance(client, str) or not 8 <= len(client) <= 80:
            raise ValueError("页面标识无效。")
        if (
            e.mode != "live"
            or not f
            or self.clock() - e.last_receive > 0.3
            or (f["build"], f.get("protocol_version")) != ("UI02", 8)
        ):
            raise ValueError("需要UI02 V8实机数据。")
        if (
            f["state"] != 4
            or 1 <= f.get("life_phase", 0) <= 8
            or e.flip.phase in ("hello", "pending", "running", "aborting")
        ):
            raise ValueError("请等待当前动作结束并恢复稳摆。")
        if not f.get("life_down_valid"):
            raise ValueError("下垂参考未就绪；请先在待机时自然下垂静置。")
        self.owner = client
        self.session = secrets.randbits(32) or 1
        self.seq = 0
        self.old_id = f["life_id"]
        self.heartbeat = self.clock()
        self.issued = False
        self.abort_sent = False
        self.sent = -1
        self.start_seq = None
        if e.trajectory.phase in ("hello", "arming", "running", "stopping"):
            e.trajectory.stop()
            self.phase = "waiting_curve"
            self.deadline = self.clock() + 4
            self.message = "先减速停止速度曲线…"
        else:
            self.phase = "hello"
            self.deadline = self.clock() + 1.5
            self.message = "正在请求受控回落…"
        self.event("request", session=self.session)

    def pulse(self, client):
        if client != self.owner:
            raise ValueError("此页面无当前任务权限。")
        self.heartbeat = self.clock()

    def cancel(self):
        if self.phase in ("waiting_curve", "hello"):
            self.phase = "error"
            self.message = "已取消，未发送回落启动。"
        elif self.phase in ("pending", "running"):
            self.phase = "aborting"
            self.abort_sent = False
            self.deadline = self.clock() + 1
            self.message = "正在中止动作；立即急停仍使用板上S0。"

    def step(self, e):
        now = self.clock()
        f = e.latest
        if self.phase not in ACTIVE:
            return None
        if (
            e.mode != "live"
            or not f
            or now - e.last_receive > 0.3
            or f["build"] != "UI02"
        ):
            self.phase = "error"
            self.message = "数据中断，停止续期；板端看门狗将停止动作。"
            return None
        if now - self.heartbeat > 0.5:
            self.cancel()
        if self.phase == "error":
            return None
        same = f.get("stream_session") == self.session
        if self.phase == "waiting_curve":
            if (
                f.get("stream_velocity_q16") == 0
                and f.get("stream_desired_q16", 0) == 0
                and e.trajectory.phase in ("idle", "done", "error")
            ):
                self.phase = "hello"
                self.sent = -1
                self.deadline = now + 1.5
            elif now > self.deadline:
                self.phase = "error"
                self.message = "曲线未能停稳，未启动回落。"
            return None
        if self.phase == "hello":
            if now > self.deadline:
                self.phase = "error"
                self.message = "握手未确认，未启动。"
                return None
            if same and f.get("stream_flags", 0) & 32:
                self.phase = "pending"
                self.sent = -1
                self.deadline = now + 1
        elif self.phase == "pending":
            if (
                f.get("life_id") != self.old_id
                and f.get("life_mode") == 1
                and 4 <= f.get("life_phase", 0) <= 8
            ):
                self.phase = "running"
                self.sent = -1
                self.deadline = now + 9
            elif (
                self.issued
                and now - self.sent > 0.06
                and same
                and f.get("stream_seq") == self.start_seq
                and f.get("life_result")
            ):
                self.phase = "error"
                self.message = REASON.get(f["life_result"], "启动被拒绝")
                return None
            elif now > self.deadline:
                self.phase = "error"
                self.message = "启动未确认，不重发启动命令。"
                return None
        if self.phase in ("running", "aborting"):
            if f.get("life_id") != self.old_id and f.get("life_phase") == 10:
                self.phase = "done"
                self.message = "已受控回到下垂，完成静止确认并关闭驱动。"
                self.event("completed")
                return None
            if f.get("life_phase") == 11:
                self.phase = "error"
                self.message = REASON.get(f.get("life_result"), "动作未完成")
                self.event("failed", reason=f.get("life_result"))
                return None
            if now > self.deadline:
                self.cancel()
        if self.phase == "aborting":
            if self.abort_sent:
                if now > self.deadline:
                    self.phase = "error"
                    self.message = "中止确认超时，请使用S0。"
                return None
            op = 9
            self.abort_sent = True
        elif self.phase == "hello":
            if now - self.sent < 0.1:
                return None
            op = 0
        elif self.phase == "pending":
            if self.issued:
                return None
            op = 7
            self.issued = True
        elif self.phase == "running":
            if now - self.sent < 0.1:
                return None
            op = 8
        else:
            return None
        self.seq = (self.seq + 1) & 65535
        self.sent = now
        if op == 7:
            self.start_seq = self.seq
        b = packet(op, self.seq, self.session)
        self.event("tx", op=op, seq=self.seq, session=self.session, hex=b.hex())
        return b

    def status(self):
        return dict(phase=self.phase, message=self.message, session=self.session)
