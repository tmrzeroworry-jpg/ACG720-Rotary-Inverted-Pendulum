"""Actual host Engine over a synthetic PTY; no FPGA or physical model."""

import os, pty, select, struct, sys, threading, time, json, tempfile, unittest, csv, io, zipfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Mac上位机"))
import server
from protocol_vendor import crc16
from history_export import HistoryExport

ROOT = Path(__file__).resolve().parents[1]


class E2E(unittest.TestCase):
    def test_flip_duplex_once_keepalive_and_record(self):
        master, slave = pty.openpty()
        path = os.ttyname(slave)
        old_ports = server.ports
        server.ports = lambda: [path]
        ended = threading.Event()
        seen = []
        buf = bytearray()
        state = dict(session=0, seq=0, started=None, ident=3)
        base = bytearray((ROOT / "tests/flip_uart_vectors.bin").read_bytes()[:137])
        start = time.monotonic()
        e = None

        def board():
            n = 0
            while not ended.wait(0.01):
                try:
                    if select.select([master], [], [], 0)[0]:
                        buf.extend(os.read(master, 4096))
                    while len(buf) >= 24:
                        if buf[:4] != b"\xaa\xc3\x01\x18":
                            del buf[0]
                            continue
                        b = bytes(buf[:24])
                        del buf[:24]
                        if crc16(b[:20]) != int.from_bytes(b[20:22], "big"):
                            continue
                        op = b[4]
                        seen.append(op)
                        state.update(
                            seq=int.from_bytes(b[6:8], "big"),
                            session=int.from_bytes(b[8:12], "big"),
                        )
                        if op == 4:
                            self.assertEqual(
                                int.from_bytes(b[12:16], "big", signed=True), -2
                            )
                            state.update(started=time.monotonic(), ident=4)
                    age = time.monotonic() - state["started"] if state["started"] else 0
                    phase = (
                        0
                        if not state["started"]
                        else 1
                        if age < 0.15
                        else 2
                        if age < 0.4
                        else 3
                        if age < 0.6
                        else 4
                        if age < 1.1
                        else 5
                    )
                    b = bytearray(base)
                    b[:8] = b"\xaa\x56\x08\xacUI02"
                    b[8:12] = n.to_bytes(4, "big")
                    b[12:16] = round((time.monotonic() - start) * 1000).to_bytes(
                        4, "big"
                    )
                    b[21] = 8 if phase in (1, 2) else 3 if phase == 3 else 4
                    b[109:127] = struct.pack(
                        ">IHBBiiH", state["session"], state["seq"], 33, 0, 0, 0, 10
                    )
                    b[127:137] = struct.pack(
                        ">BBHiH",
                        phase,
                        0,
                        state["ident"],
                        round(min(1, age / 0.6) * 26400),
                        round(age * 1000),
                    )
                    b += struct.pack(">BBHHBBHB i", 0, 0, 0, 0, 0, 0, 20000, 1, 0)
                    b += bytes(16)
                    b += struct.pack(">H", crc16(b)) + b"\r\n"
                    os.write(master, b)
                    n += 1
                except OSError:
                    pass

        def wait_for(fn, timeout=3):
            until = time.monotonic() + timeout
            while time.monotonic() < until:
                if e:
                    with e.lock:
                        if fn():
                            return
                time.sleep(0.01)
            raise AssertionError("timeout: " + str(e.status().get("flip")))

        worker = threading.Thread(target=board, daemon=True)
        worker.start()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                app = Path(tmp)
                (app / "data").mkdir()
                (app / "data/catalog.json").write_text("[]")
                e = server.Engine(app)
                e.connect(path)
                wait_for(lambda: e.latest is not None)
                with e.lock:
                    e.flip.start(e, "pageflip1", -1, 1)
                until = time.monotonic() + 3
                while time.monotonic() < until:
                    with e.lock:
                        e.flip.pulse("pageflip1")
                        if e.flip.phase in ("done", "error"):
                            break
                    time.sleep(0.08)
                self.assertEqual(e.flip.phase, "done", e.flip.message)
                self.assertEqual(seen.count(4), 1)
                self.assertGreaterEqual(seen.count(5), 3)
                e.stop()
                events = [
                    json.loads(x)
                    for x in next((app / "recordings").glob("*.commands.jsonl"))
                    .read_text()
                    .splitlines()
                ]
                self.assertTrue(
                    any(
                        x.get("event") == "completed" and x.get("scope") == "flip"
                        for x in events
                    )
                )
                self.assertGreater(
                    next((app / "recordings").glob("*.bin")).stat().st_size, 141
                )
        finally:
            if e:
                e.stop()
                e.history.pool.shutdown(wait=True)
            ended.set()
            worker.join(1)
            os.close(master)
            os.close(slave)
            server.ports = old_ports
        (ROOT / "reports/flip_serial_e2e.json").write_text(
            json.dumps(
                dict(
                    passed=True,
                    hardware_access=False,
                    transport="synthetic PTY",
                    seen_commands=seen,
                ),
                indent=2,
            )
            + "\n"
        )

    def test_v6_history_csv_preserves_flip_and_encoder_adc(self):
        def wait(m, j):
            for _ in range(300):
                s = m.status(j["id"])
                if s["status"] == "error":
                    raise AssertionError(s["error"])
                if s["status"] == "done":
                    return s
                time.sleep(0.01)
            raise AssertionError("export timeout")

        with tempfile.TemporaryDirectory() as tmp:
            app = Path(tmp)
            (app / "data").mkdir()
            (app / "recordings").mkdir()
            (app / "data/catalog.json").write_text("[]")
            data = (ROOT / "tests/flip_uart_vectors.bin").read_bytes()
            (app / "recordings/test.bin").write_bytes(data)
            m = HistoryExport(app)
            try:
                idx = wait(m, m.index("recordings:test.bin"))
                job = wait(m, m.export(idx["id"]))
                with zipfile.ZipFile(m.download(job["id"])) as z:
                    raw = z.read("samples.csv")
                    self.assertTrue(raw.startswith(b"\xef\xbb\xbf"))
                    rows = list(csv.DictReader(io.StringIO(raw.decode("utf-8-sig"))))
                    self.assertEqual(len(rows), 6)
                    r = rows[0]
                    self.assertEqual(r["flip_progress_counts"], "26405")
                    self.assertAlmostEqual(
                        float(r["flip_progress_deg"]), 26405 * 360 / 26400
                    )
                    self.assertEqual(r["flip_id"], "3")
                    self.assertEqual(r["flip_phase"], "4")
                    self.assertEqual(r["flip_elapsed_ms"], "1578")
                    self.assertEqual(r["encoder_counts_per_rev"], "1040")
                    self.assertTrue(r["potentiometer_adc_raw"])
                    self.assertEqual(z.read("raw_uart.bin"), data)
            finally:
                m.pool.shutdown(wait=True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
