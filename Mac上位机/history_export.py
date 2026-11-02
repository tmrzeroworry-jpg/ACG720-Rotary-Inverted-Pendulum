"""Read-only, asynchronous history index and Windows-friendly sample exports."""

import binascii, csv, datetime, hashlib, json, math, threading, tempfile, types, uuid, zipfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import protocol_vendor
from telemetry import HEADERS, decode_rf02, normalize

# Use the existing protocol parser with a private, equivalent C CRC implementation.
# Its globals are copied: the live serial decoder is never patched or shared.
fast_crc = lambda b: binascii.crc_hqx(b, 0xFFFF)
decode_fast = types.FunctionType(
    protocol_vendor.decode_frame.__code__,
    dict(protocol_vendor.decode_frame.__globals__, crc16=fast_crc),
)
DEFAULT_CAL = {"up": 6251, "down": 20039}
FIELDS = [
    "time_s",
    "record_time_s",
    "trial_time_s",
    "sample_interval_s",
    "missing_before",
    "record_epoch",
    "trial_id",
    "seq",
    "device_time_ms",
    "firmware",
    "state_id",
    "state_name",
    "potentiometer_adc_raw",
    "potentiometer_voltage_v",
    "pole_display_deg",
    "pole_phase_deg",
    "pole_signed_deg",
    "pole_control_error_deg",
    "encoder_counts_total",
    "encoder_counts_mod",
    "encoder_counts_per_rev",
    "arm_angle_total_deg",
    "arm_angle_mod_deg",
    "arm_velocity_deg_s",
    "flip_id",
    "flip_phase",
    "flip_result",
    "flip_progress_counts",
    "flip_progress_deg",
    "flip_elapsed_ms",
    "flip_sensor_diagnostic",
    "flip_seam_age_ms",
    "flip_sensor_fault",
    "flip_rate_counts_s",
    "flip_energy_error",
    "flip_goal_pwm",
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
    "remote_session",
    "remote_seq",
    "remote_op",
    "remote_result",
    "remote_flags",
    "remote_mode",
    "remote_cfg",
    "remote_age_ms",
    "sensor_ok",
    "reference_ok",
    "arm_home_counts",
    "arm_home_valid",
    "encoder_relative_counts_total",
    "encoder_relative_counts_mod",
    "arm_relative_angle_total_deg",
    "arm_relative_angle_mod_deg",
]


def calibration(value):
    c = {k: float((value or DEFAULT_CAL)[k]) for k in ("up", "down")}
    if not all(math.isfinite(v) and 0 <= v < 26400 for v in c.values()):
        raise ValueError("显示标定无效。")
    if not 6600 <= (c["down"] - c["up"]) % 26400 <= 19800:
        raise ValueError("显示标定的上下端过近。")
    return c


def display_pole(raw, c):
    x = (raw - c["up"]) % 26400
    d = (c["down"] - c["up"]) % 26400
    phase = 180 * x / d if x <= d else 180 + 180 * (x - d) / (26400 - d)
    return min(phase, 360 - phase), phase, (phase + 180) % 360 - 180


def scan(path, size, quality, progress=lambda n: None):
    """Stream a fixed byte-prefix; preserve valid frame bytes and measured intervals."""
    buf = bytearray()
    offset = 0
    read = 0
    previous = None
    epoch = 0
    timeline = 0.0
    digest = hashlib.sha256()
    quality.update(
        valid_frames=0, bad_crc=0, discarded_bytes=0, auxiliary=0, missing_frames=0
    )
    with Path(path).open("rb") as src:
        while read < size:
            block = src.read(min(1024 * 1024, size - read))
            if not block:
                raise ValueError("原始记录已被截短，请重新刷新记录。")
            digest.update(block)
            read += len(block)
            buf.extend(block)
            while len(buf) >= 4:
                length = HEADERS.get(bytes(buf[:4]))
                if length is None:
                    skip = buf.find(b"\xaa", 1)
                    if skip < 0:
                        skip = max(1, len(buf) - 3)
                    del buf[:skip]
                    offset += skip
                    quality["discarded_bytes"] += skip
                    continue
                if len(buf) < length:
                    break
                raw = bytes(buf[:length])
                if raw[-2:] != b"\r\n" or fast_crc(raw[:-4]) != int.from_bytes(
                    raw[-4:-2], "big"
                ):
                    quality["bad_crc"] += 1
                    del buf[0]
                    offset += 1
                    quality["discarded_bytes"] += 1
                    continue
                here = offset
                del buf[:length]
                offset += length
                if length not in (64, 104, 108, 113, 131, 141, 156, 172):
                    quality["auxiliary"] += 1
                    continue
                try:
                    f = decode_rf02(raw) if length == 64 else decode_fast(raw)
                except (ValueError, IndexError, OverflowError):
                    quality["discarded_bytes"] += length
                    continue
                dt = None
                missing = 0
                if previous is not None:
                    sd = (f["seq"] - previous["seq"]) & 0xFFFFFFFF
                    td = (f["uptime_ms"] - previous["uptime_ms"]) & 0xFFFFFFFF
                    if (
                        f["build_id"] != previous["build_id"]
                        or sd > 0x80000000
                        or td > 0x80000000
                    ):
                        epoch += 1
                    else:
                        dt = td / 1000.0
                        timeline += dt
                        missing = max(0, sd - 1)
                        quality["missing_frames"] += missing
                f.update(
                    epoch=epoch,
                    record_time_s=timeline,
                    sample_interval_s=dt,
                    missing_before=missing,
                )
                quality["valid_frames"] += 1
                previous = f
                yield raw, f, here
            progress(read / size if size else 1)
    quality.update(
        snapshot_bytes=size,
        source_sha256=digest.hexdigest(),
        pending_tail_bytes=len(buf),
    )


class HistoryExport:
    def __init__(self, root):
        self.root = Path(root)
        self.lock = threading.RLock()
        self.jobs = {}
        self.pool = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="history_export"
        )

    def records(self):
        try:
            labels = {
                v["id"]: v["label"]
                for v in json.loads((self.root / "data/catalog.json").read_text())
            }
        except (OSError, ValueError, KeyError):
            labels = {}
        out = []
        for bucket in ("recordings", "data"):
            for p in (self.root / bucket).glob("*.bin"):
                if not p.is_file() or p.stat().st_size == 0:
                    continue
                stat = p.stat()
                out.append(
                    dict(
                        id=bucket + ":" + p.name,
                        name=p.name,
                        label=labels.get(p.name, p.name),
                        bytes=stat.st_size,
                        modified=stat.st_mtime,
                        location="串口记录"
                        if bucket == "recordings"
                        else "已导入／内置记录",
                    )
                )
        return sorted(out, key=lambda v: v["modified"], reverse=True)

    def source(self, ident):
        if not isinstance(ident, str) or ":" not in ident:
            raise ValueError("无效记录。")
        bucket, name = ident.split(":", 1)
        if (
            bucket not in ("recordings", "data")
            or not name
            or Path(name).name != name
            or "\\" in name
            or not name.endswith(".bin")
        ):
            raise ValueError("无效记录。")
        p = (self.root / bucket / name).resolve()
        folder = (self.root / bucket).resolve()
        if p.parent != folder or not p.is_file():
            raise ValueError("记录不存在。")
        return p

    def status(self, ident):
        with self.lock:
            if ident not in self.jobs:
                raise ValueError("任务不存在，请重新选择记录。")
            return {k: v for k, v in self.jobs[ident].items() if not k.startswith("_")}

    def update(self, ident, **kw):
        with self.lock:
            self.jobs[ident].update(kw)

    def submit(self, kind, fn, **args):
        ident = uuid.uuid4().hex
        with self.lock:
            self.jobs[ident] = dict(id=ident, kind=kind, status="queued", progress=0)

        def run():
            self.update(ident, status="running")
            try:
                fn(ident, **args)
                self.update(ident, status="done", progress=100)
            except Exception as e:
                self.update(ident, status="error", error=str(e))

        self.pool.submit(run)
        return self.status(ident)

    def index(self, record):
        p = self.source(record)
        stat = p.stat()
        if not stat.st_size:
            raise ValueError("记录为空。")
        return self.submit(
            "index", self._index, record=record, path=p, size=stat.st_size
        )

    def _index(self, job, record, path, size):
        quality = {}
        groups = {}
        first = None
        last = None
        for raw, f, offset in scan(
            path, size, quality, lambda v: self.update(job, progress=round(v * 99))
        ):
            key = "%d:%s:%d" % (f["epoch"], f["build_id"], f["trial"])
            if first is None:
                first = f["record_time_s"]
            last = f["record_time_s"]
            g = groups.setdefault(
                key,
                dict(
                    key=key,
                    epoch=f["epoch"],
                    build=f["build_id"],
                    trial=f["trial"],
                    frames=0,
                    first_s=last,
                    last_s=last,
                    start_state=f["state"],
                    end_state=f["state"],
                ),
            )
            g.update(frames=g["frames"] + 1, last_s=last, end_state=f["state"])
        if not groups:
            raise ValueError("没有CRC正确的可识别遥测帧。")
        for g in groups.values():
            g["duration_s"] = g["last_s"] - g["first_s"]
        result = dict(
            record=record,
            name=path.name,
            snapshot_bytes=size,
            duration_s=(last - first),
            quality=quality,
            trials=list(groups.values()),
        )
        self.update(job, result=result, _path=path)

    def export(self, index_job, selection="all", display_calibration=None):
        with self.lock:
            j = self.jobs.get(index_job)
            if not j or j["kind"] != "index" or j["status"] != "done":
                raise ValueError("请先完成记录分析。")
            index = dict(j["result"])
            path = j["_path"]
        if selection != "all" and selection not in [g["key"] for g in index["trials"]]:
            raise ValueError("无效试验。")
        c = calibration(display_calibration)
        return self.submit(
            "export", self._export, index=index, path=path, selection=selection, cal=c
        )

    def _export(self, job, index, path, selection, cal):
        outdir = self.root / "exports"
        outdir.mkdir(exist_ok=True)
        quality = {}
        rows = 0
        first = None
        last = None
        observed = {}
        states = set()
        firmware = set()
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = (
            "pendulum_"
            + stamp
            + "_"
            + ("all" if selection == "all" else "trial" + selection.split(":")[-1])
            + "_"
            + job[:6]
            + ".zip"
        )
        with tempfile.TemporaryDirectory(prefix="prepare_", dir=outdir) as tmp:
            temp = Path(tmp)
            with (
                (temp / "samples.csv").open(
                    "w", encoding="utf-8-sig", newline=""
                ) as dest,
                (temp / "raw_uart.bin").open("wb") as original,
            ):
                writer = csv.DictWriter(dest, fieldnames=FIELDS, lineterminator="\r\n")
                writer.writeheader()
                for raw, f, offset in scan(
                    path,
                    index["snapshot_bytes"],
                    quality,
                    lambda v: self.update(job, progress=round(v * 85)),
                ):
                    key = "%d:%s:%d" % (f["epoch"], f["build_id"], f["trial"])
                    observed.setdefault(key, f["record_time_s"])
                    if selection != "all" and key != selection:
                        continue
                    v = normalize(f)
                    if first is None:
                        first = f["record_time_s"]
                    last = f["record_time_s"]
                    rev = v["counts_per_rev"]
                    count = f["arm_position"]
                    pole, phase, signed = (None, None, None)
                    if v["pose_valid"]:
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
                            pole, phase, signed = display_pole(f["raw"], cal)
                        else:
                            phase = v["pole"] % 360
                            pole = min(phase, 360 - phase)
                            signed = (phase + 180) % 360 - 180
                    row = dict(
                        time_s=f["record_time_s"] - first,
                        record_time_s=f["record_time_s"],
                        trial_time_s=f["record_time_s"] - observed[key],
                        sample_interval_s=f["sample_interval_s"],
                        missing_before=f["missing_before"],
                        record_epoch=f["epoch"],
                        trial_id=f["trial"],
                        seq=f["seq"],
                        device_time_ms=f["uptime_ms"],
                        firmware=f["build_id"],
                        state_id=f["state"],
                        state_name="FLIP360"
                        if f["build_id"]
                        in ("TR02", "TR03", "TR04", "TR05", "TR06", "UI01", "UI02")
                        and f["state"] == 8
                        else protocol_vendor.STATES.get(f["state"], "unknown"),
                        potentiometer_adc_raw=f["raw"],
                        potentiometer_voltage_v=f["raw"] * 0.000125
                        if f["build_id"]
                        in (
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
                        else None,
                        pole_display_deg=pole,
                        pole_phase_deg=phase,
                        pole_signed_deg=signed,
                        pole_control_error_deg=v["pole"],
                        encoder_counts_total=count,
                        encoder_counts_mod=count % rev,
                        encoder_counts_per_rev=rev,
                        arm_angle_total_deg=count * 360 / rev
                        if v["sensor_ok"]
                        else None,
                        arm_angle_mod_deg=(count % rev) * 360 / rev
                        if v["sensor_ok"]
                        else None,
                        arm_velocity_deg_s=v["arm_rate"],
                        sensor_ok=int(v["sensor_ok"]),
                        reference_ok=int(v["reference_ok"]),
                    )
                    row.update(
                        {
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
                        }
                    )
                    row.update(
                        flip_id=f.get("flip_id"),
                        flip_phase=f.get("flip_phase"),
                        flip_result=f.get("flip_result"),
                        flip_progress_counts=f.get("flip_progress"),
                        flip_progress_deg=f["flip_progress"] * 360 / 26400
                        if f.get("flip_progress") is not None
                        else None,
                        flip_elapsed_ms=f.get("flip_elapsed_ms"),
                    )
                    row.update(
                        {
                            k: f.get(k)
                            for k in (
                                "flip_sensor_diagnostic",
                                "flip_seam_age_ms",
                                "flip_sensor_fault",
                                "flip_rate_counts_s",
                                "flip_energy_error",
                                "flip_goal_pwm",
                            )
                        }
                    )
                    row.update(
                        {
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
                        }
                    )
                    home = f.get("arm_home_counts")
                    home_valid = f.get("arm_home_valid", False)
                    relative = count - home if home_valid else None
                    row.update(
                        arm_home_counts=home,
                        arm_home_valid=int(home_valid) if home is not None else None,
                        encoder_relative_counts_total=relative,
                        encoder_relative_counts_mod=relative % rev
                        if relative is not None
                        else None,
                        arm_relative_angle_total_deg=relative * 360 / rev
                        if relative is not None and v["sensor_ok"]
                        else None,
                        arm_relative_angle_mod_deg=(relative % rev) * 360 / rev
                        if relative is not None and v["sensor_ok"]
                        else None,
                    )
                    writer.writerow(
                        {
                            k: (
                                ""
                                if x is None
                                else format(x, ".9f").rstrip("0").rstrip(".")
                                if isinstance(x, float)
                                else x
                            )
                            for k, x in row.items()
                        }
                    )
                    original.write(raw)
                    rows += 1
                    states.add(f["state"])
                    firmware.add(f["build_id"])
            if rows == 0:
                raise ValueError("所选试验没有有效样本。")
            if quality["source_sha256"] != index["quality"]["source_sha256"]:
                raise ValueError("记录内容发生变化，请重新分析后导出。")
            meta = dict(
                format_version=2,
                created=datetime.datetime.now().astimezone().isoformat(),
                source_file=path.name,
                source_id=index["record"],
                source_snapshot_bytes=index["snapshot_bytes"],
                selection=selection,
                rows=rows,
                duration_s=last - first,
                firmware=sorted(firmware),
                states=sorted(states),
                quality=quality,
                display_calibration=cal,
                data_policy="all received valid samples; no interpolation, smoothing or resampling",
                timing="time_s starts at first exported sample; trial_time_s starts at first observed sample of that trial; reboot downtime is unknown and not added",
                counts_per_rev="WEXF/WEXH/TR01/TR02: 1040; legacy1058 (same as host)",
                csv_encoding="UTF-8 with BOM, CRLF",
                raw_scope="selected valid telemetry frames with their original CRC; not the entire source recording",
            )
            meta["files"] = {
                p.name: dict(
                    bytes=p.stat().st_size,
                    sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
                )
                for p in (temp / "samples.csv", temp / "raw_uart.bin")
            }
            (temp / "metadata.json").write_text(
                json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
            (temp / "README_数据说明.txt").write_text(README, encoding="utf-8-sig")
            self.update(job, progress=90)
            out = outdir / filename
            with zipfile.ZipFile(out, "x", zipfile.ZIP_DEFLATED) as z:
                for p in sorted(temp.iterdir()):
                    z.write(p, p.name)
        self.update(
            job,
            result=dict(
                filename=filename,
                rows=rows,
                duration_s=last - first,
                bytes=out.stat().st_size,
                download="/api/history/download?id=" + job,
            ),
            _output=out,
        )

    def download(self, ident):
        with self.lock:
            j = self.jobs.get(ident)
            if not j or j["kind"] != "export" or j["status"] != "done":
                raise ValueError("导出尚未完成。")
            return j["_output"]


README = """旋转倒立摆——Windows实验数据包

解压即可使用，不需要运行Mac上位机。samples.csv为UTF-8带BOM、CRLF换行，Windows Excel可直接打开；Python可用pandas.read_csv(..., encoding="utf-8-sig")，MATLAB可用readtable。

文件：samples.csv逐帧数据；raw_uart.bin为所选有效遥测帧原始字节；metadata.json为来源、采样数量、质量、显示标定和SHA256。

制作视频：按time_s（秒）对齐，不要把每行当作一个视频帧。未自动插值、补帧、平滑或改采样率；可自行按30/60fps取样。设备重启时实际断电间隔未知，record_epoch可区分；time_s只累计已知设备间隔，不虚构停机时长。

主要列：
time_s：本次导出首样本起算的秒数；record_time_s：原记录内累计已知时间；trial_time_s：该试验首次观测样本起算时间（若中途接入，不等于实际启动时刻）。
device_time_ms、seq：原固件时间戳、帧序号；record_epoch：记录内重启/固件切换区间；trial_id：原试验编号；state_id/state_name：固件阶段。
potentiometer_adc_raw：电位器原始ADC码；potentiometer_voltage_v：仅WEXF/WEXH/TR01/TR02按ADS1115±4.096V档位换算的输入电压，其余版本留空。
pole_display_deg：上位机显示倾角，直立0、下垂180；pole_phase_deg：0～360且保留左右方向，用于绘制摆杆；pole_signed_deg：同一相位的±180表示；pole_control_error_deg：原软件控制偏角，与显示标定不同。
新增home/relative字段仅在V4遥测提供有效停泊位时填写；原编码器字段保留复位坐标。
encoder_counts_total：原编码器有符号累加计数；encoder_counts_mod：一圈内计数；encoder_counts_per_rev：当前换算刻度，WEXF/WEXH/TR01/TR02: 1040，旧版WE20为1058。
arm_angle_total_deg：连续累加角度，适合位移曲线和连续旋转视频；arm_angle_mod_deg：0～360方位；arm_velocity_deg_s：原遥测速度换算为度/秒，不是电机指令。
flip_id/flip_phase/flip_result：TR02单次翻转编号、阶段与结果；flip_progress_counts/flip_progress_deg：沿所选方向展开的摆杆累计转角，26400码等于360°；flip_elapsed_ms：该动作经过时间。旧版这些列留空。
sensor_ok/reference_ok：有效性标志。无效角度留空，不补造。sample_interval_s是相邻已接收帧间隔，missing_before是此前缺帧数。

物理标定：WEXF/WEXH/TR01/TR02显示角度采用上位机独立的两端映射（参数见metadata），默认直立端6251沿用软件参考、下垂端20039来自静止记录。它不是精密角度标定，也不改变运动控制。实际传感器死区、非线性和噪声仍可能存在。保留原始ADC与原控制角度供自行复算。

所选试验可能因中途连接或中断而不完整，不能仅凭编号认定从起摆到结束均已记录。统计质量和原始来源见metadata。
"""
