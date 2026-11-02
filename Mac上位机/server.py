#!/usr/bin/env python3
"""UI02 local host: telemetry, replay, exports and explicitly leased control."""

import argparse, bisect, collections, csv, datetime, glob, hashlib, io, json, os, select, signal, threading, time, uuid, webbrowser
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit, parse_qs
from telemetry import Decoder, normalize, parse_record, TrialMetrics
from history_export import HistoryExport
from trajectory import Trajectory
from return_motion import ReturnMotion
from flip import Flip
from controls import Controls

ROOT = Path(__file__).resolve().parent


def ports():
    return sorted(
        set(
            p
            for pattern in (
                "/dev/cu.usbserial*",
                "/dev/cu.wchusbserial*",
                "/dev/ttyUSB*",
                "/dev/ttyACM*",
            )
            for p in glob.glob(pattern)
        )
    )


class Engine:
    def __init__(self, root=ROOT):
        self.root = Path(root)
        self.action_lock = threading.Lock()
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.worker = None
        self.generation = 0
        self.index = 0
        self.mode = "idle"
        self.latest = None
        self.rows = collections.deque(maxlen=6000)
        self.baud = 115200
        self.metrics = TrialMetrics()
        self.quality = {}
        self.error = ""
        self.label = "未连接"
        self.last_receive = 0
        self.recording = None
        self.frames = []
        self.times = []
        self.position = 0.0
        self.speed = 1.0
        self.playing = False
        self.next_frame = 0
        self.record_id = None
        self.catalog = json.loads((self.root / "data/catalog.json").read_text())
        self.control = Controls()
        self.history = HistoryExport(self.root)
        self.trajectory = Trajectory()
        self.flip = Flip()
        self.return_motion = ReturnMotion()

    def stop(self):
        with self.lock:
            self.control.disconnect()
            self.trajectory.disconnect()
            self.flip.disconnect()
            self.return_motion.disconnect()
        self.stop_event.set()
        if self.worker and self.worker.is_alive():
            self.worker.join(timeout=3)
        if self.worker and self.worker.is_alive():
            raise RuntimeError("旧采集线程仍在关闭，请稍后再试。")
        with self.lock:
            self.playing = False
            self.worker = None

    def reset(self, mode, label):
        self.generation += 1
        self.mode = mode
        self.label = label
        self.index = 0
        self.latest = None
        self.rows.clear()
        self.metrics = TrialMetrics()
        self.quality = {}
        self.error = ""
        self.last_receive = 0
        self.recording = None

    def emit(self, frame, t):
        v = normalize(frame)
        v["t"] = t
        self.index += 1
        v["i"] = self.index
        if v["build"] != "RF02" or v["seq"] % 10 == 0:
            self.rows.append(v)
        self.latest = v
        self.metrics.add(v)
        self.last_receive = time.monotonic()
        if (
            self.mode == "live"
            and v.get("build") == "UI02"
            and self.return_motion.phase == "idle"
        ):
            self.return_motion.message = "稳摆后可点击受控回落；S0仍为立即急停。"

    def replay_load(self, record_id):
        entry = next((x for x in self.catalog if x["id"] == record_id), None)
        if not entry:
            raise ValueError("未知记录。")
        data = (self.root / "data" / record_id).read_bytes()
        frames, times, q = parse_record(data)
        self.stop()
        with self.lock:
            self.reset("replay", entry["label"])
            self.frames = frames
            self.times = times
            self.position = 0.0
            self.next_frame = 0
            self.record_id = record_id
            self.quality = q
            self.playing = False
            self.speed = 1.0
            self._seek(0)
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self.replay_loop, daemon=True)
        self.worker.start()

    def _seek(self, position):
        self.position = max(0.0, min(float(position), self.times[-1]))
        self.generation += 1
        self.rows.clear()
        self.metrics = TrialMetrics()
        self.index = 0
        end = bisect.bisect_right(self.times, self.position)
        # Metrics include all already-observed BAL frames; charts retain at most60s.
        for i in range(end):
            if self.times[i] >= self.position - 60:
                self.emit(self.frames[i], self.times[i])
            else:
                self.metrics.add(normalize(self.frames[i]))
        self.next_frame = end

    def replay_action(self, action, position=None, speed=None):
        with self.lock:
            if self.mode != "replay":
                raise ValueError("请先选择历史记录。")
            if action == "play":
                self.playing = True
            elif action == "pause":
                self.playing = False
            elif action == "seek":
                self._seek(position)
            elif action == "speed":
                if float(speed) not in (0.25, 0.5, 1.0, 2.0, 4.0):
                    raise ValueError("无效播放速度。")
                self.speed = float(speed)
            else:
                raise ValueError("未知播放操作。")

    def replay_loop(self):
        previous = time.monotonic()
        while not self.stop_event.wait(0.02):
            now = time.monotonic()
            dt = now - previous
            previous = now
            with self.lock:
                if not self.playing:
                    continue
                self.position = min(self.times[-1], self.position + dt * self.speed)
                while (
                    self.next_frame < len(self.frames)
                    and self.times[self.next_frame] <= self.position
                ):
                    j = self.next_frame
                    self.emit(self.frames[j], self.times[j])
                    self.next_frame += 1
                if self.position >= self.times[-1]:
                    self.playing = False

    def connect(self, port, baud=115200):
        if baud not in (115200, 1000000):
            raise ValueError("只支持115200或1000000。")
        if port not in ports():
            raise ValueError("串口已不存在，请刷新后选择USB串口。")
        self.stop()
        import termios, fcntl

        lease_path = Path("/tmp") / (
            "inverted_pendulum_rx_" + Path(port).name + ".lock"
        )
        lease = lease_path.open("a")
        fd = None
        try:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            fcntl.ioctl(fd, termios.TIOCEXCL)
            a = termios.tcgetattr(fd)
            a[0] = a[1] = a[3] = 0
            a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            a[4] = a[5] = termios.B115200
            a[6][termios.VMIN] = a[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, a)
            if baud == 1000000:
                import sys, array

                if sys.platform == "darwin":
                    fcntl.ioctl(fd, 0x80045402, array.array("I", [baud]))
                else:
                    a[4] = a[5] = termios.B1000000
                    termios.tcsetattr(fd, termios.TCSANOW, a)
            stamp = (
                datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                + "_"
                + uuid.uuid4().hex[:5]
            )
            path = self.root / "recordings" / ("live_" + stamp + ".bin")
            path.parent.mkdir(exist_ok=True)
            output = path.open("xb", buffering=0)
        except Exception:
            if fd is not None:
                os.close(fd)
            lease.close()
            raise
        with self.lock:
            self.reset("live", port)
            self.recording = path.name
            self.baud = baud
            self.flip.message = "已连接；先点击起摆并稳定直立，再执行一次翻转。"
            self.trajectory.message = "已连接；编辑不会自动启动运动。"
        self.stop_event = threading.Event()
        self.worker = threading.Thread(
            target=self.serial_loop, args=(fd, lease, output, path), daemon=True
        )
        self.worker.start()

    def serial_loop(self, fd, lease, output, path):
        d = Decoder()
        start = time.monotonic()
        count = 0
        reason = "用户断开"
        previous = None
        timeline = 0.0
        tx_pending = b""
        tx_count = 0
        event_cursor = 0
        flip_event_cursor = 0
        return_event_cursor = 0
        control_event_cursor = 0
        command_log = path.with_suffix(".commands.jsonl").open("w", encoding="utf-8")
        try:
            while not self.stop_event.is_set():
                with self.lock:
                    cmd = self.control.step(self)
                    if cmd is None:
                        cmd = self.return_motion.step(self)
                    if cmd is None:
                        cmd = self.flip.step(self)
                    if cmd is None:
                        cmd = self.trajectory.step(self)
                    if cmd:
                        tx_pending += cmd
                    events = (
                        [
                            dict(scope="control", **v)
                            for v in self.control.events[control_event_cursor:]
                        ]
                        + [
                            dict(scope="return", **v)
                            for v in self.return_motion.events[return_event_cursor:]
                        ]
                        + self.trajectory.events[event_cursor:]
                        + [
                            dict(scope="flip", **v)
                            for v in self.flip.events[flip_event_cursor:]
                        ]
                    )
                    event_cursor = len(self.trajectory.events)
                    flip_event_cursor = len(self.flip.events)
                    return_event_cursor = len(self.return_motion.events)
                    control_event_cursor = len(self.control.events)
                for event in events:
                    command_log.write(json.dumps(event, ensure_ascii=False) + "\n")
                if events:
                    command_log.flush()
                if tx_pending:
                    try:
                        n = os.write(fd, tx_pending)
                        tx_count += n
                        tx_pending = tx_pending[n:]
                    except BlockingIOError:
                        pass
                    if len(tx_pending) > 240:
                        raise OSError("串口发送拥塞，终止曲线流")
                if not select.select([fd], [], [], 0.01)[0]:
                    continue
                try:
                    b = os.read(fd, 16384)
                except BlockingIOError:
                    continue
                if not b:
                    raise OSError("串口已断开（读取到EOF）")
                output.write(b)
                count += len(b)
                with self.lock:
                    for f in d.feed(b):
                        if previous is None:
                            timeline = time.monotonic() - start
                        else:
                            delta = (
                                f["uptime_ms"] - previous["uptime_ms"]
                            ) & 0xFFFFFFFF
                            timeline += (
                                delta / 1000
                                if f["epoch"] == previous["epoch"]
                                and delta < 0x80000000
                                else 0.01
                            )
                        self.emit(f, timeline)
                        previous = f
                    self.quality = d.quality()
        except (OSError, ValueError) as e:
            reason = "串口中断：" + str(e)
            with self.lock:
                self.error = reason
        finally:
            command_log.close()
            output.close()
            os.close(fd)
            lease.close()
            path.with_suffix(".session.json").write_text(
                json.dumps(
                    dict(
                        port=self.label,
                        bytes=count,
                        reason=reason,
                        duration_s=time.monotonic() - start,
                        sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                        quality=d.quality(),
                        transmits_data=bool(tx_count),
                        tx_bytes=tx_count,
                        format=str(self.baud) + "8N1",
                    ),
                    ensure_ascii=False,
                    indent=2,
                )
                + "\n"
            )

    def import_record(self, data, name):
        if len(data) > 32 * 1024 * 1024:
            raise ValueError("单文件上限32MB。")
        f, t, q = parse_record(data)
        name = Path(name).name[:100] or "导入记录"
        record_id = "import_" + uuid.uuid4().hex[:12] + ".bin"
        (self.root / "data" / record_id).write_bytes(data)
        entry = dict(
            id=record_id,
            label=name,
            kind="导入的串口记录",
            frames=len(f),
            duration_s=t[-1],
            quality=q,
            sha256=hashlib.sha256(data).hexdigest(),
        )
        with self.lock:
            self.catalog.append(entry)
            (self.root / "data/catalog.json").write_text(
                json.dumps(self.catalog, ensure_ascii=False, indent=2) + "\n"
            )
        return record_id

    def status(self, after=0):
        with self.lock:
            age = time.monotonic() - self.last_receive if self.last_receive else None
            stale = self.mode == "live" and (age is None or age > 0.5)
            new = [r for r in self.rows if r["i"] > after]
            if len(new) > 1000 and after > 0:
                new = new[-1000:]
            return dict(
                control=self.control.status(),
                return_motion=self.return_motion.status(),
                flip=self.flip.status(),
                trajectory=self.trajectory.status(),
                generation=self.generation,
                mode=self.mode,
                label=self.label,
                latest=self.latest,
                stale=stale,
                age=age,
                quality=self.quality,
                error=self.error,
                frames=new,
                trials=self.metrics.output(),
                recording=self.recording,
                baud=self.baud,
                record_kind=next(
                    (
                        x.get("kind", "")
                        for x in self.catalog
                        if x["id"] == self.record_id
                    ),
                    "",
                ),
                replay=dict(
                    id=self.record_id,
                    position=self.position,
                    duration=self.times[-1] if self.times else 0,
                    playing=self.playing,
                    speed=self.speed,
                ),
                now=time.time(),
            )

    def csv(self):
        with self.lock:
            rows = self.metrics.output()
        f = io.StringIO()
        f.write("\ufeff")
        keys = [
            "epoch",
            "build",
            "trial",
            "route",
            "pole_filter",
            "arm_filter",
            "balance",
            "balance_lower_bound",
            "rms",
            "arm_range",
            "samples",
            "stage",
            "reason",
        ]
        out = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        out.writeheader()
        out.writerows(rows)
        return f.getvalue().encode("utf-8")


class Handler(BaseHTTPRequestHandler):
    server_version = "WEXH-Mac"

    def log_message(self, *args):
        pass

    def send(self, status, data, ctype="application/json; charset=utf-8", headers=None):
        if isinstance(data, (dict, list)):
            data = json.dumps(data, ensure_ascii=False, allow_nan=False).encode()
        if isinstance(data, str):
            data = data.encode()
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'",
        )
        for k, v in (headers or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(data)

    def host_ok(self):
        return self.headers.get("Host", "") in (
            f"127.0.0.1:{self.server.server_port}",
            f"localhost:{self.server.server_port}",
        )

    def download(self, path):
        with path.open("rb") as source:
            self.send_response(200)
            self.send_header("Content-Type", "application/zip")
            self.send_header("Content-Length", str(path.stat().st_size))
            self.send_header(
                "Content-Disposition", 'attachment; filename="' + path.name + '"'
            )
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            while True:
                block = source.read(256 * 1024)
                if not block:
                    break
                self.wfile.write(block)

    def do_GET(self):
        if not self.host_ok():
            self.send(403, {"error": "仅允许本机访问。"})
            return
        e = self.server.engine
        p = urlsplit(self.path)
        q = parse_qs(p.query)
        try:
            if p.path == "/api/status":
                self.send(200, e.status(max(0, int(q.get("after", ["0"])[0]))))
            elif p.path == "/api/catalog":
                self.send(200, e.catalog)
            elif p.path == "/api/health":
                self.send(
                    200,
                    dict(
                        app="RIP_UI02_LIVE_MAC",
                        root=str(e.root),
                        PID=os.getpid(),
                        features=[
                            "remote-controls-ui01",
                            "history-export-v1",
                            "live-trajectory-tr01",
                            "flip",
                            "controlled-return",
                        ],
                    ),
                )
            elif p.path == "/favicon.ico":
                self.send(204, b"", "image/x-icon")
            elif p.path == "/api/ports":
                self.send(200, ports())
            elif p.path == "/api/export":
                self.send(
                    200,
                    e.csv(),
                    "text/csv; charset=utf-8",
                    {"Content-Disposition": 'attachment; filename="trial_metrics.csv"'},
                )
            elif p.path == "/api/history":
                self.send(200, e.history.records())
            elif p.path == "/api/history/job":
                self.send(200, e.history.status(q.get("id", [""])[0]))
            elif p.path == "/api/history/download":
                self.download(e.history.download(q.get("id", [""])[0]))
            elif p.path in (
                "/trajectory.js",
                "/trajectory_math.js",
                "/trajectory.css",
                "/flip.js",
                "/return_motion.js",
                "/controls.js",
                "/controls.css",
            ):
                file = p.path[1:]
                self.send(
                    200,
                    (e.root / "web" / file).read_bytes(),
                    "text/javascript; charset=utf-8"
                    if file.endswith(".js")
                    else "text/css; charset=utf-8",
                )
            elif p.path in ("/", "/app.js", "/style.css"):
                file = {
                    "/": "index.html",
                    "/app.js": "app.js",
                    "/style.css": "style.css",
                }[p.path]
                ctype = {
                    "/": "text/html; charset=utf-8",
                    "/app.js": "text/javascript; charset=utf-8",
                    "/style.css": "text/css; charset=utf-8",
                }[p.path]
                self.send(200, (e.root / "web" / file).read_bytes(), ctype)
            else:
                self.send(404, {"error": "未找到。"})
        except (ValueError, KeyError, OSError) as ex:
            self.send(400, {"error": str(ex)})

    def do_POST(self):
        with self.server.engine.action_lock:
            self.handle_post()

    def handle_post(self):
        origin = self.headers.get("Origin")
        if not self.host_ok() or (
            origin
            and origin
            not in (
                f"http://127.0.0.1:{self.server.server_port}",
                f"http://localhost:{self.server.server_port}",
            )
        ):
            self.send(403, {"error": "拒绝跨站请求。"})
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if size < 0 or size > 32 * 1024 * 1024:
                raise ValueError("单次请求上限32MB。")
            body = self.rfile.read(size)
            p = urlsplit(self.path)
            e = self.server.engine
            if p.path == "/api/import":
                name = parse_qs(p.query).get("name", ["导入记录"])[0]
                ident = e.import_record(body, name)
                self.send(200, {"id": ident})
                return
            data = json.loads(body or b"{}")
            if p.path == "/api/history/index":
                self.send(200, e.history.index(data["id"]))
                return
            if p.path == "/api/history/export":
                self.send(
                    200,
                    e.history.export(
                        data["index_job"],
                        data.get("selection", "all"),
                        data.get("calibration"),
                    ),
                )
                return
            if p.path == "/api/control/claim":
                with e.lock:
                    e.control.claim(e, data.get("client"))
                self.send(200, {"ok": True})
                return
            elif p.path == "/api/control/action":
                with e.lock:
                    e.control.action(
                        e, data.get("client"), data.get("action"), data.get("value", 0)
                    )
                self.send(200, {"ok": True})
                return
            elif p.path == "/api/control/heartbeat":
                with e.lock:
                    e.control.pulse(data.get("client"))
                self.send(200, {"ok": True})
                return
            elif p.path == "/api/control/stop":
                with e.lock:
                    e.control.stop(e)
                self.send(200, {"ok": True})
                return
            if p.path == "/api/return/start":
                with e.lock:
                    e.control.special(e, data.get("client"))
                    e.return_motion.start(e, data.get("client"))
                self.send(200, {"ok": True})
                return
            elif p.path == "/api/return/heartbeat":
                with e.lock:
                    e.return_motion.pulse(data.get("client"))
                self.send(200, {"ok": True})
                return
            elif p.path == "/api/return/cancel":
                with e.lock:
                    e.return_motion.cancel()
                self.send(200, {"ok": True})
                return
            if p.path == "/api/flip/start":
                with e.lock:
                    e.control.special(e, data.get("client"))
                    e.flip.start(
                        e,
                        data.get("client"),
                        data.get("direction"),
                        data.get("variant", 0),
                    )
                self.send(200, {"ok": True})
                return
            elif p.path == "/api/flip/heartbeat":
                with e.lock:
                    e.flip.pulse(data.get("client"))
                self.send(200, {"ok": True})
                return
            elif p.path == "/api/flip/cancel":
                with e.lock:
                    e.flip.cancel()
                self.send(200, {"ok": True})
                return
            if p.path == "/api/trajectory/validate":
                with e.lock:
                    c = e.trajectory.update(data)
                self.send(200, c)
                return
            elif p.path == "/api/trajectory/start":
                with e.lock:
                    if (
                        e.flip.phase in ("hello", "pending", "running", "aborting")
                        or 1 <= (e.latest or {}).get("life_phase", 0) <= 8
                    ):
                        raise ValueError("请先等待当前动作结束。")
                    e.control.special(e, data.get("client"))
                    e.trajectory.start(e, data.get("client"))
            elif p.path == "/api/trajectory/heartbeat":
                with e.lock:
                    e.trajectory.pulse(data.get("client"))
            elif p.path == "/api/trajectory/stop":
                with e.lock:
                    e.trajectory.stop()
            elif p.path == "/api/connect":
                e.connect(data["port"], data.get("baud", 115200))
            elif p.path == "/api/disconnect":
                e.stop()
                with e.lock:
                    e.mode = "idle"
                    e.generation += 1
                    e.latest = None
                    e.rows.clear()
                    e.label = "已断开"
            elif p.path == "/api/replay/load":
                e.replay_load(data["id"])
            elif p.path == "/api/replay/action":
                e.replay_action(data["action"], data.get("position"), data.get("speed"))
            else:
                self.send(404, {"error": "未知操作。"})
                return
            self.send(200, {"ok": True})
        except (ValueError, KeyError, TypeError, OSError, RuntimeError) as ex:
            self.send(400, {"error": str(ex)})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8777)
    ap.add_argument("--open", action="store_true")
    ap.add_argument("--demo", action="store_true")
    a = ap.parse_args()
    e = Engine()
    if a.demo:
        e.replay_load(e.catalog[0]["id"])
        e.replay_action("play")
    server = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    server.engine = e
    url = f"http://127.0.0.1:{server.server_port}"
    print(url, flush=True)

    def stop(signum, frame):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    if a.open:
        webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        e.stop()
        server.server_close()


if __name__ == "__main__":
    main()
