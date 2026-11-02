import csv, io, json, sys, tempfile, time, unittest, zipfile, hashlib, threading, urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from history_export import HistoryExport, scan, display_pole, fast_crc
from telemetry import Decoder
from protocol_vendor import crc16
import server

APP = Path(__file__).resolve().parents[1]
VECTOR = (APP / "tests/wexf_uart_vectors.bin").read_bytes()


def change(raw, values):
    b = bytearray(raw)
    for (offset, size), v in values.items():
        b[offset : offset + size] = v.to_bytes(size, "big", signed=v < 0)
    b[104:106] = fast_crc(b[:104]).to_bytes(2, "big")
    return bytes(b)


def wait(manager, job):
    until = time.monotonic() + 15
    while time.monotonic() < until:
        s = manager.status(job["id"])
        if s["status"] == "error":
            raise AssertionError(s["error"])
        if s["status"] == "done":
            return s
        time.sleep(0.01)
    raise AssertionError("job did not finish")


class HistoryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        for d in ("recordings", "data"):
            (self.root / d).mkdir()
        (self.root / "data/catalog.json").write_text("[]")
        self.path = self.root / "recordings/test.bin"
        self.path.write_bytes(VECTOR)
        self.m = HistoryExport(self.root)

    def tearDown(self):
        self.m.pool.shutdown(wait=True)
        self.tmp.cleanup()

    def test_csv_zip_original_values_windows_encoding_and_source_immutable(self):
        before = self.path.read_bytes()
        index = wait(self.m, self.m.index("recordings:test.bin"))
        job = wait(
            self.m, self.m.export(index["id"], index["result"]["trials"][0]["key"])
        )
        with zipfile.ZipFile(self.m.download(job["id"])) as z:
            data = z.read("samples.csv")
            self.assertTrue(data.startswith(b"\xef\xbb\xbf"))
            self.assertIn(b"\r\n", data)
            rows = list(csv.DictReader(io.StringIO(data.decode("utf-8-sig"))))
            self.assertEqual(len(rows), 6)
            r = rows[0]
            self.assertEqual(r["encoder_counts_total"], "-1040")
            self.assertEqual(r["encoder_counts_mod"], "0")
            self.assertEqual(r["arm_angle_total_deg"], "-360")
            self.assertEqual(r["potentiometer_adc_raw"], "6223")
            self.assertAlmostEqual(float(rows[-1]["time_s"]), 0.05)
            self.assertEqual(rows[0]["sample_interval_s"], "")
            self.assertAlmostEqual(float(r["potentiometer_voltage_v"]), 6223 * 0.000125)
            self.assertEqual(z.read("raw_uart.bin"), VECTOR)
            meta = json.loads(z.read("metadata.json"))
            self.assertEqual(meta["rows"], 6)
            self.assertEqual(
                meta["files"]["samples.csv"]["sha256"], hashlib.sha256(data).hexdigest()
            )
        self.assertEqual(self.path.read_bytes(), before)

    def test_trial_selection_reboot_and_missing_frames(self):
        a = VECTOR[:108]
        frames = [
            change(a, {(8, 4): 10, (12, 4): 100, (16, 4): 1}),
            change(a, {(8, 4): 12, (12, 4): 120, (16, 4): 2}),
            change(a, {(8, 4): 0, (12, 4): 0, (16, 4): 1}),
        ]
        self.path.write_bytes(b"".join(frames))
        idx = wait(self.m, self.m.index("recordings:test.bin"))
        self.assertEqual(len(idx["result"]["trials"]), 3)
        self.assertEqual(idx["result"]["quality"]["missing_frames"], 1)
        selected = idx["result"]["trials"][2]["key"]
        done = wait(self.m, self.m.export(idx["id"], selected))
        with zipfile.ZipFile(self.m.download(done["id"])) as z:
            rows = list(
                csv.DictReader(io.StringIO(z.read("samples.csv").decode("utf-8-sig")))
            )
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["record_epoch"], "1")
            self.assertEqual(rows[0]["sample_interval_s"], "")
            self.assertEqual(rows[0]["time_s"], "0")
            self.assertEqual(z.read("raw_uart.bin"), frames[-1])

    def test_fixed_prefix_snapshot_can_grow_and_rejects_modified_prefix(self):
        idx = wait(self.m, self.m.index("recordings:test.bin"))
        with self.path.open("ab") as f:
            f.write(VECTOR)
        done = wait(self.m, self.m.export(idx["id"]))
        self.assertEqual(done["result"]["rows"], 6)
        self.path.write_bytes(VECTOR[108:] + VECTOR[:108] + VECTOR)
        j = self.m.export(idx["id"])
        until = time.monotonic() + 5
        while time.monotonic() < until and self.m.status(j["id"])["status"] not in (
            "error",
            "done",
        ):
            time.sleep(0.01)
        self.assertEqual(self.m.status(j["id"])["status"], "error")

    def test_crc_recovery_path_validation_and_calibration(self):
        self.assertEqual(fast_crc(VECTOR), crc16(VECTOR))
        bad = bytearray(VECTOR[:108])
        bad[40] ^= 4
        self.path.write_bytes(b"noise" + bad + VECTOR + b"\xaaV")
        q = {}
        out = list(scan(self.path, self.path.stat().st_size, q))
        self.assertEqual(len(out), 6)
        self.assertEqual(q["bad_crc"], 1)
        self.assertEqual(q["pending_tail_bytes"], 2)
        for v in (
            "recordings:../test.bin",
            "other:test.bin",
            "recordings:..\\test.bin",
            "recordings:catalog.json",
        ):
            with self.assertRaises(ValueError):
                self.m.source(v)
        (self.root / "recordings/escape.bin").symlink_to(
            APP / "tests/wexf_uart_vectors.bin"
        )
        with self.assertRaises(ValueError):
            self.m.source("recordings:escape.bin")
        self.assertEqual(display_pole(6251, {"up": 6251, "down": 20039})[0], 0)
        self.assertEqual(display_pole(20039, {"up": 6251, "down": 20039})[0], 180)
        idx = wait(self.m, self.m.index("recordings:test.bin"))
        with self.assertRaises(ValueError):
            self.m.export(idx["id"], display_calibration={"up": 0, "down": 1})

    def test_http_index_export_and_download_without_touching_engine(self):
        e = server.Engine(self.root)
        srv = server.ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
        srv.engine = e
        t = threading.Thread(target=srv.serve_forever, daemon=True)
        t.start()
        url = "http://127.0.0.1:" + str(srv.server_port)

        def get(route):
            return json.load(urllib.request.urlopen(url + route))

        def post(route, data):
            return json.load(
                urllib.request.urlopen(
                    urllib.request.Request(
                        url + route,
                        data=json.dumps(data).encode(),
                        headers={"Content-Type": "application/json"},
                    )
                )
            )

        try:
            self.assertEqual(get("/api/history")[0]["id"], "recordings:test.bin")
            j = post("/api/history/index", {"id": "recordings:test.bin"})
            idx = wait(e.history, j)
            j = post(
                "/api/history/export", {"index_job": idx["id"], "selection": "all"}
            )
            done = wait(e.history, j)
            with urllib.request.urlopen(url + done["result"]["download"]) as r:
                self.assertEqual(r.headers["Content-Type"], "application/zip")
                self.assertEqual(
                    zipfile.ZipFile(io.BytesIO(r.read())).read("raw_uart.bin"), VECTOR
                )
            self.assertEqual(e.mode, "idle")
            self.assertIsNone(e.worker)
        finally:
            srv.shutdown()
            srv.server_close()
            t.join()
            e.history.pool.shutdown(wait=True)


if __name__ == "__main__":
    unittest.main()
