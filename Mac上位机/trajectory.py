"""TR01 live trajectory validation + reference-only serial commands.

Explicit web control starts swing-up. Explicit browser START is accepted only in BAL.
No start/resume on reconnect, page refresh, visibility changes or telemetry GET.
"""

import bisect, math, secrets, struct, time
from protocol_vendor import crc16

HARD_V = 360.0
HARD_A = 720.0
MAX_A_STEP = 136
MAX_V_Q16 = 68157
MAX_POINTS = 16
DEFAULT_V = 180.0
DEFAULT_A = 360.0
Q = 65536 * 1040 / (360 * 1000)


def finite(x, name):
    if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
        raise ValueError(name + "必须是有限数值。")
    return float(x)


def validate_curve(data):
    vmax = finite(data.get("vmax", DEFAULT_V), "速度上限")
    amax = finite(data.get("amax", DEFAULT_A), "加速度上限")
    if not 1 <= vmax <= HARD_V or not 6 <= amax <= HARD_A:
        raise ValueError("速度上限1～360°/s，加速度上限6～720°/s²。")
    points = data.get("points")
    if not isinstance(points, list) or not 2 <= len(points) <= MAX_POINTS:
        raise ValueError("曲线需要2～16个控制点。")
    raw = []
    clipped = 0
    for p in points:
        if not isinstance(p, list) or len(p) != 2:
            raise ValueError("每个点应为[时间秒,速度°/s]。")
        t = finite(p[0], "时间")
        v = finite(p[1], "速度")
        if abs(v) > 10000:
            raise ValueError("速度输入异常。")
        if t < 0 or t > 120:
            raise ValueError("时间范围0～120秒。")
        if raw and t - raw[-1][0] < 0.099999999:
            raise ValueError("时间必须递增，间隔至少0.1秒；同一时刻只能有一个速度值。")
        nv = max(-vmax, min(vmax, v))
        clipped += int(nv != v)
        raw.append([t, nv])
    if raw[0][0] != 0:
        raise ValueError("第一个点的时间必须为0。")
    if raw[-1][0] < 1:
        raise ValueError("总时长至少1秒。")
    if abs(raw[0][1]) > 1e-9 or abs(raw[-1][1]) > 1e-9:
        raise ValueError("首尾速度必须为0，便于平滑起停。")
    peak = max(abs((b[1] - a[1]) / (b[0] - a[0])) for a, b in zip(raw, raw[1:]))
    if peak > amax + 1e-7:
        raise ValueError(
            "加速度过快：%.2f°/s²，当前上限%.2f°/s²。请拉长过渡时间。" % (peak, amax)
        )
    astep = max(1, min(MAX_A_STEP, math.floor(amax * Q / 1000)))
    # Validate quantized hardware capability too, instead of promising rounded-up acceleration.
    actual_a = astep * 1000 / Q
    if peak > actual_a + 1e-7:
        raise ValueError(
            "受FPGA定点步长限制，实际加速度上限为%.2f°/s²，请稍微减缓曲线。" % actual_a
        )
    return dict(
        points=raw,
        vmax=vmax,
        amax=actual_a,
        acc_step=astep,
        peak_acceleration=peak,
        clipped_points=clipped,
        duration=raw[-1][0],
    )


def value_at(curve, t):
    p = curve["points"]
    if t <= 0:
        return p[0][1]
    if t >= p[-1][0]:
        return 0.0
    i = bisect.bisect_right([v[0] for v in p], t) - 1
    a, b = p[i], p[i + 1]
    return a[1] + (b[1] - a[1]) * (t - a[0]) / (b[0] - a[0])


def packet(op, seq, session, velocity=0, acc_step=11, ttl=300):
    q = max(-MAX_V_Q16, min(MAX_V_Q16, round(velocity * Q)))
    b = struct.pack(
        ">4sBBHIiHH", b"\xaa\xc3\x01\x18", op, 0, seq & 65535, session, q, acc_step, ttl
    )
    return b + struct.pack(">H", crc16(b)) + b"\r\n"


def example_points(duration=15.0):
    if not 1.5 <= duration <= 120:
        raise ValueError("16点示例总时长需为1.5～120秒。")
    return [
        [
            round(duration * i / 15, 2),
            round(150 * math.sin(4 * math.pi * i / 15), 2) if i not in (0, 15) else 0,
        ]
        for i in range(16)
    ]


class Trajectory:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.curve = validate_curve(dict(points=example_points()))
        self.session = 0
        self.seq = 0
        self.phase = "idle"
        self.message = "先启用网页控制并起摆，稳摆后再开始。"
        self.heartbeat = 0.0
        self.sent_at = 0.0
        self.started = 0.0
        self.deadline = 0.0
        self.samples = []
        self.revision = 0
        self.events = []
        self.trial = None
        self.epoch = None
        self.owner = None
        self.stop_seq = None
        self.acceleration = None
        self.overspeed_since = None
        self.edit_revisions = {}

    def event(self, name, **fields):
        self.events.append(dict(at=self.clock(), event=name, **fields))

    def disconnect(self):
        self.phase = "idle"
        self.session = 0
        self.samples = []
        self.event("disconnect")
        self.message = "未连接；不会自动恢复运动。"

    def update(self, data):
        if (
            self.phase in ("hello", "arming", "running", "stopping")
            and data.get("client") != self.owner
        ):
            raise ValueError("另一页面正在控制曲线，请先在原页面停止。")
        editor = data.get("client", "")
        revision = data.get("editor_revision", 0)
        if type(revision) is not int or revision < 0:
            raise ValueError("无效曲线修订号。")
        if revision < self.edit_revisions.get(editor, -1):
            raise ValueError("旧版编辑请求已被更新的曲线替代。")
        c = validate_curve(data)
        if self.phase in ("hello", "arming", "running", "stopping") and (
            c["vmax"] != self.curve["vmax"] or c["acc_step"] != self.curve["acc_step"]
        ):
            raise ValueError("运行中限值固定，请先减速停止，再修改速度和加速度上限。")
        if (
            self.phase == "running"
            and c["duration"] <= self.clock() - self.started + 0.2
        ):
            raise ValueError("新曲线必须保留至少0.2秒未来时间。")
        self.edit_revisions[editor] = revision
        if len(self.edit_revisions) > 64:
            self.edit_revisions = {editor: revision}
        self.curve = c
        self.revision += 1
        self.event("curve_update", revision=self.revision, curve=c)
        return c

    def start(self, engine, client):
        if not isinstance(client, str) or not 8 <= len(client) <= 80:
            raise ValueError("无效页面标识。")
        self.require_live(engine)
        if self.phase in ("hello", "arming", "running", "stopping"):
            raise ValueError("任务尚未停止。")
        f = engine.latest
        if f["state"] != 4:
            raise ValueError("请先点击起摆，进入稳摆后再开始曲线。")
        if f.get("stream_velocity_q16", 0) != 0:
            raise ValueError("参考速度尚未归零，请等待减速完成。")
        self.owner = client
        self.stop_seq = None
        self.session = secrets.randbits(32) or 1
        self.seq = 0
        self.phase = "hello"
        self.heartbeat = self.clock()
        self.sent_at = -1
        self.deadline = self.clock() + 1
        self.samples = []
        self.acceleration = None
        self.overspeed_since = None
        self.trial = f["trial"]
        self.epoch = f["epoch"]
        self.event("start_requested", session=self.session)
        self.message = "正在请求板端确认…"

    def require_live(self, e):
        f = e.latest
        if e.mode != "live" or not f or self.clock() - e.last_receive > 0.3:
            raise ValueError("没有新鲜的实时串口数据。")
        if (f["build"], f.get("protocol_version")) != ("UI02", 8):
            raise ValueError("当前固件不支持曲线下发，需要UI02 V8。")

    def pulse(self, client):
        # A page refresh is not an authorization to resume a running task.
        if self.owner is not None and client != self.owner:
            raise ValueError("该页面没有当前曲线的心跳权限。")
        self.heartbeat = self.clock()

    def stop(self, reason="用户停止"):
        if self.phase in ("hello", "arming", "running"):
            self.phase = "stopping"
            self.sent_at = -1
            self.message = reason + "，正在限加速度减速。"
            self.deadline = self.clock() + max(
                2.0, min(70.0, self.curve["vmax"] / self.curve["amax"] + 0.75)
            )
            self.event(
                "soft_stop",
                reason=reason,
                confirmation_budget_s=self.deadline - self.clock(),
            )

    def step(self, e):
        now = self.clock()
        f = e.latest
        if self.phase in ("idle", "done", "error"):
            return None
        if (
            not f
            or now - e.last_receive > 0.3
            or e.mode != "live"
            or f["build"] not in ("TR01", "TR02", "TR03", "TR04", "UI02")
        ):
            self.phase = "error"
            self.message = "遥测中断，停止发送；FPGA看门狗将减速。"
            self.event("telemetry_timeout")
            return None
        if f["epoch"] != self.epoch or f["trial"] != self.trial or f["state"] != 4:
            self.phase = "error"
            self.message = "已退出本次稳摆，曲线取消，不自动重启。"
            self.event("balance_exit")
            return None
        if now - self.heartbeat > 0.5:
            self.stop("网页心跳中断")
        if self.phase in ("hello", "arming") and now > self.deadline:
            self.phase = "error"
            self.message = "板端未确认，任务没有开始；不会自动重试。"
            self.event("ack_timeout")
            return None
        same = f.get("stream_session") == self.session
        flags = f.get("stream_flags", 0)
        if self.phase == "hello" and same and flags & 32:
            self.phase = "arming"
            self.sent_at = -1
            self.deadline = now + 1
        elif (
            self.phase == "arming"
            and same
            and flags & 2
            and not flags & 4
            and f.get("stream_result") == 0
        ):
            self.phase = "running"
            self.started = now
            self.message = "运行中：修改曲线后将在后续时刻生效。"
            self.event("board_started")
            self.sent_at = -1
        elif self.phase in ("running", "stopping") and (
            not same or not flags & 2 or flags & 4 or flags & 16
        ):
            self.phase = "error"
            self.message = "板端退出曲线模式或通信超时，需手动重新开始。"
            self.event("board_cancelled", flags=flags)
            return None
        if same and flags & 8 and self.phase in ("arming", "running"):
            self.phase = "error"
            self.message = "板端拒绝命令（状态码%d），原始按键功能仍可用。" % f.get(
                "stream_result", -1
            )
            self.event("rejected")
            return None
        elapsed = (
            max(0.0, now - self.started)
            if self.phase in ("running", "stopping")
            else 0.0
        )
        if self.phase == "running" and elapsed >= self.curve["duration"]:
            self.stop("曲线完成")
        if (
            self.phase == "stopping"
            and same
            and self.stop_seq is not None
            and f.get("stream_velocity_q16") == 0
            and f.get("stream_desired_q16") == 0
            and ((f.get("stream_seq", 0) - self.stop_seq) & 65535) < 32768
            and now - self.sent_at > 0.02
        ):
            self.phase = "done"
            self.message = "已停止；保持当前位置的平衡参考。"
            self.event("finished")
            return None
        if self.phase == "stopping" and now > self.deadline:
            self.phase = "error"
            self.message = "减速确认超时，已停止发送，请查看板端状态。"
            return None
        if self.phase == "running":
            rate = f.get("arm_rate")
            if rate is not None and abs(rate) > max(
                self.curve["vmax"] * 1.5, self.curve["vmax"] + 30
            ):
                if self.overspeed_since is None:
                    self.overspeed_since = now
                elif now - self.overspeed_since > 0.3:
                    self.stop("实际速度持续超出跟踪范围")
            else:
                self.overspeed_since = None
        if now - self.sent_at < 0.05:
            return None
        op = (
            0
            if self.phase == "hello"
            else 1
            if self.phase == "arming"
            else 3
            if self.phase == "stopping"
            else 2
        )
        target = value_at(self.curve, elapsed) if op == 2 else 0.0
        self.seq = (self.seq + 1) & 65535
        self.sent_at = now
        if op == 3 and self.stop_seq is None:
            self.stop_seq = self.seq
        if op in (2, 3):
            self.samples.append(
                dict(
                    t=elapsed,
                    target=target,
                    limited=f.get("stream_velocity_q16", 0) / Q,
                    actual=f.get("arm_rate"),
                )
            )
            if len(self.samples) > 3000:
                self.samples = self.samples[-3000:]
        b = packet(op, self.seq, self.session, target, self.curve["acc_step"])
        if len(self.samples) > 2:
            tail = self.samples[-1]
            previous = next(
                (p for p in reversed(self.samples[:-1]) if tail["t"] - p["t"] >= 0.1),
                None,
            )
            if (
                previous
                and tail.get("actual") is not None
                and previous.get("actual") is not None
            ):
                self.acceleration = (tail["actual"] - previous["actual"]) / (
                    tail["t"] - previous["t"]
                )
        self.event(
            "tx",
            op=op,
            seq=self.seq,
            session=self.session,
            target=target,
            acc_step=self.curve["acc_step"],
            hex=b.hex(),
        )
        return b

    def status(self):
        elapsed = (
            max(0.0, self.clock() - self.started)
            if self.phase in ("running", "stopping")
            else 0.0
        )
        return dict(
            phase=self.phase,
            message=self.message,
            curve=self.curve,
            elapsed=elapsed,
            revision=self.revision,
            estimated_acceleration=self.acceleration,
            session=self.session,
            seq=self.seq,
            samples=self.samples[-1000:],
        )
