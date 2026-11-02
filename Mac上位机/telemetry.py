"""Streaming CRC-checked telemetry. No hardware transmission functions."""

import math
from protocol_vendor import crc16, decode_frame, decode_config_frame

HEADERS = {
    b"\xaaV\x08\xac": 172,
    b"\xaaV\x07\x9c": 156,
    b"\xaaV\x06\x8d": 141,
    b"\xaaV\x05\x83": 131,
    b"\xaaV\x04q": 113,
    b"\xaaV\x02h": 104,
    b"\xaaV\x03l": 108,
    b"\xaaW\x01\x10": 16,
    b"\xaaX\x01\x14": 20,
    b"\xaaX\x02\x34": 52,
    b"\xaaY\x02\x40": 64,
}
KNOWN = {
    "T54A",
    "T55A",
    "T60R",
    "WT01",
    "PD01",
    "PD02",
    "HD01",
    "WE20",
    "WEXF",
    "WEXH",
    "TR01",
    "TR02",
    "TR03",
    "TR04",
    "TR05",
    "TR06",
    "UI01",
    "UI02",
    "LQ01",
    "LQ02",
    "LQ03",
    "LQ04",
    "LQ05",
    "LQ06",
    "LQ07",
    "6EU1",
    "7EU1",
    "WEU1",
    "6EU2",
    "7EU2",
    "WEU2",
    "RF02",
}


def decode_rf02(b):
    """Read-only RF02 adapter; CRC already checked by the streaming decoder.

    This frame sends raw ADC, not the internal guarded controller angle.
    It does not send firmware balance timing or speed/statistics fields.
    """
    if b[4:8] != b"RF02":
        raise ValueError("RF02 identity")

    def u(k, n):
        return int.from_bytes(b[k : k + n], "big")

    def s(k, n):
        return int.from_bytes(b[k : k + n], "big", signed=True)

    profile = b[39]
    v = profile - 1
    pole = (v >> 2) & 3
    arm = v & 3
    if not 1 <= profile <= 59 or pole > 2 or arm > 2:
        raise ValueError("RF02 profile")
    names = {0: "original", 1: "fast_1e", 2: "quiet_1e"}
    return dict(
        build_id="RF02",
        seq=u(8, 4),
        uptime_ms=u(8, 4),
        trial=u(16, 4),
        profile=profile,
        state=b[36],
        fault=b[37],
        flags=b[38],
        raw=u(26, 2),
        error=(u(26, 2) - u(44, 2) + 13200) % 26400 - 13200,
        trim=s(46, 2),
        arm_position=s(28, 4),
        arm_target_counts=s(40, 4),
        arm_rate=0,
        command=s(32, 2),
        angle_up=u(44, 2),
        angle_down=0,
        balance_ms=0,
        reversals=None,
        saturation_ms=None,
        source_profile=(v >> 4) + 1,
        pole_filter=names[pole],
        arm_filter=names[arm],
        remote_status=b[49],
        balance_observed=True,
    )


STAGES = {
    0: "初始化",
    1: "待机",
    2: "自动起摆",
    3: "捕获",
    4: "稳摆",
    5: "停止过渡",
    6: "故障",
    7: "减速",
    8: "只读诊断",
}
REASONS = {
    12: "起摆/回落任务未完成或已中止",
    11: "同向翻转未完成或已中止",
    0: "无故障",
    1: "传感器异常",
    2: "横臂行程限制",
    3: "起摆 / 捕获超时",
    4: "摆杆角度超限",
    5: "正常到时结束",
    6: "换向保护",
    7: "横臂转速保护",
    8: "指令累计保护",
    9: "标定异常",
    10: "运算异常",
}


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.valid = 0
        self.bad_crc = 0
        self.discarded = 0
        self.auxiliary = 0
        self.previous = None
        self.missing = 0
        self.epoch = 0
        self.config = None
        self.rf_balance = {}
        self.wexf_balance = {}

    def feed(self, data):
        self.buffer.extend(data)
        out = []
        while len(self.buffer) >= 4:
            size = HEADERS.get(bytes(self.buffer[:4]))
            if size is None:
                del self.buffer[0]
                self.discarded += 1
                continue
            if len(self.buffer) < size:
                break
            b = bytes(self.buffer[:size])
            if b[-2:] != b"\r\n" or crc16(b[:-4]) != int.from_bytes(b[-4:-2], "big"):
                self.bad_crc += 1
                del self.buffer[0]
                self.discarded += 1
                continue
            if size in (64, 104, 108, 113, 131, 141, 156, 172):
                try:
                    f = decode_rf02(b) if size == 64 else decode_frame(b)
                except (ValueError, IndexError, OverflowError):
                    self.bad_crc += 1
                    del self.buffer[:size]
                    continue
                if self.previous:
                    seq_delta = (f["seq"] - self.previous["seq"]) & 0xFFFFFFFF
                    time_delta = (
                        f["uptime_ms"] - self.previous["uptime_ms"]
                    ) & 0xFFFFFFFF
                    if (
                        f["build_id"] != self.previous["build_id"]
                        or seq_delta > 0x80000000
                        or time_delta > 0x80000000
                    ):
                        self.epoch += 1
                    elif 1 < seq_delta < 0x80000000:
                        self.missing += seq_delta - 1
                f["epoch"] = self.epoch
                if f["build_id"] in (
                    "WEXF",
                    "WEXH",
                    "TR01",
                    "TR02",
                    "TR03",
                    "TR04",
                    "TR05",
                    "TR06",
                    "UI01",
                    "UI02",
                ):
                    # trial_statistics.v is a wrapping uint16 millisecond
                    # counter, not the saturating controller state timer.
                    # First value and each modulo delta give a lower bound:
                    # pre-connection or >65.536s gaps may hide full wraps.
                    key = (self.epoch, f["build_id"], f["trial"])
                    raw = f["balance_ms"]
                    previous = self.wexf_balance.get(key)
                    total = (
                        raw
                        if previous is None
                        else previous[1] + ((raw - previous[0]) & 65535)
                    )
                    self.wexf_balance[key] = (raw, total)
                    f["balance_counter_ms"] = raw
                    f["balance_ms"] = total
                    f["balance_lower_bound"] = True
                if size == 64:
                    key = (self.epoch, f["trial"])
                    ms = self.rf_balance.get(key, 0)
                    if f["state"] == 4:
                        ms += 1
                    self.rf_balance[key] = ms
                    f["balance_ms"] = ms
                if f["build_id"] in ("PD01", "PD02") and self.config:
                    f["bank_configuration"] = self.config
                out.append(f)
                self.previous = f
                self.valid += 1
            else:
                self.auxiliary += 1
                if size == 52 or (
                    size == 20 and self.previous and self.previous["build_id"] != "IRDG"
                ):
                    try:
                        self.config = decode_config_frame(b)
                    except (ValueError, IndexError):
                        pass
            del self.buffer[:size]
        return out

    def quality(self):
        return dict(
            valid=self.valid,
            bad_crc=self.bad_crc,
            discarded=self.discarded,
            auxiliary=self.auxiliary,
            missing=self.missing,
            pending_bytes=len(self.buffer),
        )


def normalize(f):
    wexf = f["build_id"] in (
        "WEXF",
        "WEXH",
        "TR01",
        "TR02",
        "TR03",
        "TR04",
        "TR05",
        "TR06",
        "UI01",
        "UI02",
    )
    counts_per_rev = 1040 if wexf else 1058
    known = f["build_id"] in KNOWN
    reference = known and 0 < f["angle_up"] < 26400 and f["angle_up"] != f["angle_down"]
    sensor = (
        bool(f["flags"] & 16) and bool(f["flags"] & 8) and not bool(f["flags"] & 32)
    )
    pose_valid = reference and sensor
    error = f["error"] - f["trim"]
    target = f.get("arm_target_counts")
    source = f.get("source_profile")
    if source is None and f["build_id"] == "WT01":
        source = f["profile"]
    routes = {1: "WT01 · PD", 2: "WT01 · LQR 20ms", 3: "WT01 · LQR 5ms", 4: "本地 LQR"}
    route = routes.get(source, f["build_id"] + " · S" + str(f["profile"]))
    if f["build_id"].startswith("LQ"):
        route = f["build_id"] + " · S" + str(f["profile"])
    filter_names = {"original": "原滤波", "fast_1e": "1€ 快速", "quiet_1e": "1€ 平滑"}
    pf = filter_names.get(f.get("pole_filter"), "固件原配置")
    af = filter_names.get(f.get("arm_filter"), "固件原配置")
    stage = STAGES.get(f["state"], "未知阶段")
    if wexf and f["state"] == 2 and f["balance_ms"] > 0:
        stage = "自动重起摆"
    if f["fault"] == 5 and f["state"] in (1, 5):
        stage = "正常完成"
    elif f.get("stop_latched") and f["trial"] and f["fault"] == 0:
        stage = "停止锁存"
    if (
        f["build_id"] in ("TR02", "TR03", "TR04", "TR05", "TR06", "UI01", "UI02")
        and f["state"] == 8
    ):
        stage = "同向翻转"
    if f["build_id"] in ("TR05", "TR06", "UI01", "UI02") and f["state"] == 9:
        stage = "快速起摆" if f.get("life_mode") == 0 else "受控回到下垂"
    reason = REASONS.get(f["fault"], "故障 " + str(f["fault"]))
    if stage == "停止锁存":
        reason = "停止锁存，需重新启动"
    return dict(
        build=f["build_id"],
        seq=f["seq"],
        epoch=f.get("epoch", 0),
        trial=f["trial"],
        uptime=f["uptime_ms"] / 1000,
        state=f["state"],
        stage=stage,
        fault=f["fault"],
        reason=reason,
        pole=error * 360 / 26400 if pose_valid else None,
        arm=((f["arm_position"] * 360 / counts_per_rev + 180) % 360 - 180)
        if sensor
        else None,
        arm_error=(f["arm_position"] - target) * 360 / counts_per_rev
        if target is not None and sensor
        else None,
        target=((target * 360 / counts_per_rev + 180) % 360 - 180)
        if target is not None and sensor
        else None,
        arm_unwrapped=f["arm_position"] * 360 / counts_per_rev if sensor else None,
        target_unwrapped=target * 360 / counts_per_rev
        if target is not None and sensor
        else None,
        target_counts=target,
        counts_per_rev=counts_per_rev,
        **{
            k: f.get(k)
            for k in (
                "life_phase",
                "life_result",
                "life_id",
                "life_elapsed_ms",
                "life_mode",
                "life_turns",
                "life_down",
                "life_down_valid",
                "life_rate",
                "life_sensor_fault",
                "life_primary_core_fault",
                "life_primary_core_state",
                "life_seam_age_ms",
                "life_adc_age_ms",
                "life_good_age_ms",
                "life_guard_flags",
                "life_guard_waiting",
                "life_guard_negative",
                "life_guard_retreat",
            )
        },
        flip_phase=f.get("flip_phase"),
        flip_result=f.get("flip_result"),
        flip_id=f.get("flip_id"),
        flip_progress=f.get("flip_progress"),
        flip_elapsed_ms=f.get("flip_elapsed_ms"),
        stream_session=f.get("stream_session"),
        stream_seq=f.get("stream_seq"),
        stream_flags=f.get("stream_flags", 0),
        stream_result=f.get("stream_result"),
        stream_velocity_q16=f.get("stream_velocity_q16"),
        stream_desired_q16=f.get("stream_desired_q16"),
        stream_age_ms=f.get("stream_age_ms"),
        **{
            k: f.get(k)
            for k in (
                "remote_session",
                "remote_seq",
                "remote_op",
                "remote_result",
                "remote_flags",
                "remote_mode",
                "remote_cfg",
                "remote_age_ms",
            )
        },
        protocol_version=f.get("protocol_version"),
        arm_home_counts=f.get("arm_home_counts"),
        arm_home_valid=f.get("arm_home_valid", False),
        arm_rate=f["arm_rate"] * 360 / counts_per_rev
        if sensor and f["build_id"] != "RF02"
        else None,
        command=f["command"],
        raw=f["raw"],
        encoder=f["arm_position"],
        up=(f["angle_up"] + f["trim"]) % 26400,
        balance=f["balance_ms"] / 1000,
        duration=None
        if wexf
        else 20
        if f["build_id"] in ("WE20", "HD01", "RF02")
        else 10,
        balance_observed=f.get("balance_observed", False),
        balance_lower_bound=f.get("balance_lower_bound", False),
        route=route,
        pole_filter=pf,
        arm_filter=af,
        profile=f["profile"],
        sensor_ok=sensor,
        reference_ok=reference,
        pose_valid=pose_valid,
        known=known,
        firmware_reversals=f["reversals"],
        saturation_ms=f["saturation_ms"],
        warning=(
            f"{f['build_id']}：横臂按1040计数/圈显示，稳摆不限时。任务模式及拨码未随串口发送，以板上操作为准；曲线显示固件实际目标。"
            if wexf
            else "RF02只读显示：摆角由原始ADC重建；计时为本次收到的稳摆样本时长。启动须使用RF02控制端，本页不发送启动或心跳。"
            if f["build_id"] == "RF02"
            else "当前为红外只读固件，不提供倒立参考；红外调试保持暂停。"
            if f["build_id"] == "IRDG"
            else "未识别的固件版本，仅展示原始字段。"
            if not known
            else "传感器数据或软件参考尚未就绪。"
            if not pose_valid
            else ""
        ),
    )


def parse_record(data):
    d = Decoder()
    frames = d.feed(data)
    if not frames:
        raise ValueError("文件中没有可识别且CRC正确的遥测帧。请选择串口原始.bin文件。")
    times = [0.0]
    for a, b in zip(frames, frames[1:]):
        delta = (b["uptime_ms"] - a["uptime_ms"]) & 0xFFFFFFFF
        times.append(
            times[-1]
            + (
                delta / 1000
                if delta < 0x80000000 and a["build_id"] == b["build_id"]
                else 0.01
            )
        )
    return frames, times, d.quality()


class TrialMetrics:
    def __init__(self):
        self.rows = {}

    def add(self, f):
        if not f["trial"] or f["build"] == "IRDG":
            return
        k = (f["epoch"], f["build"], f["trial"])
        r = self.rows.setdefault(
            k,
            dict(
                epoch=k[0],
                build=k[1],
                trial=k[2],
                route=f["route"],
                pole_filter=f["pole_filter"],
                arm_filter=f["arm_filter"],
                samples=0,
                sum_sq=0.0,
                lo=None,
                hi=None,
                balance=0.0,
                balance_lower_bound=f.get("balance_lower_bound", False),
                rms=None,
                arm_range=None,
                stage=f["stage"],
                reason=f["reason"],
            ),
        )
        r.update(
            balance=max(r["balance"], f["balance"]),
            stage=f["stage"],
            reason=f["reason"],
        )
        if f["state"] == 4 and f["pole"] is not None:
            r["samples"] += 1
            r["sum_sq"] += f["pole"] ** 2
            r["rms"] = math.sqrt(r["sum_sq"] / r["samples"])
            if f["arm_error"] is not None:
                r["lo"] = (
                    f["arm_error"] if r["lo"] is None else min(r["lo"], f["arm_error"])
                )
                r["hi"] = (
                    f["arm_error"] if r["hi"] is None else max(r["hi"], f["arm_error"])
                )
                r["arm_range"] = r["hi"] - r["lo"]

    def output(self):
        return list(self.rows.values())[-50:][::-1]
