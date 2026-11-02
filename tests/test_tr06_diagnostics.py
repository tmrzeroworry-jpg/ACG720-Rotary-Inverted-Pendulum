import unittest, sys, struct
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Mac上位机"))
from protocol_vendor import crc16, decode_frame
from telemetry import normalize


class Diagnostics(unittest.TestCase):
    def frame(self, state, phase, p):
        b = bytearray((ROOT / "tests/life_uart_vectors.bin").read_bytes()[:156])
        b[21] = state
        b[137] = phase
        b[138] = 4
        b[48:52] = struct.pack(">I", p)
        b[52:56] = struct.pack(">I", (2 << 24) | (170 << 16) | 160)
        b[56:60] = struct.pack(">I", 4 | 16)
        b[152:154] = crc16(b[:152]).to_bytes(2, "big")
        return decode_frame(b)

    def test_frozen_timeout_details(self):
        f = self.frame(1, 11, 0x40060101)
        self.assertEqual(f["life_sensor_fault"], 1)
        self.assertEqual(f["life_primary_core_fault"], 1)
        self.assertEqual(f["life_primary_core_state"], 6)
        self.assertEqual(f["life_seam_age_ms"], 160)
        self.assertEqual(f["life_good_age_ms"], 170)
        self.assertEqual(f["life_adc_age_ms"], 2)
        self.assertTrue(f["life_guard_waiting"])
        self.assertTrue(f["life_guard_negative"])
        v = normalize(f)
        self.assertEqual(v["life_sensor_fault"], 1)
        self.assertEqual(v["build"], "TR06")

    def test_legacy_or_steady_terms_not_reinterpreted(self):
        for state, phase, p in [(2, 11, 255), (4, 11, 0x40060101), (1, 11, 1)]:
            self.assertNotIn("life_sensor_fault", self.frame(state, phase, p))

    def test_active_diagnostics(self):
        self.assertEqual(self.frame(9, 2, 0x40000000)["life_sensor_fault"], 0)


class DiagnosticExport(unittest.TestCase):
    def test_error_frame_csv_retains_primary_reason(self):
        import tempfile, time, zipfile, csv, io
        from history_export import HistoryExport

        b = bytearray((ROOT / "tests/life_uart_vectors.bin").read_bytes()[:156])
        b[21] = 1
        b[22] = 1
        b[137] = 11
        b[138] = 4
        b[48:52] = struct.pack(">I", 0x40060101)
        b[52:56] = struct.pack(">I", 160)
        b[56:60] = struct.pack(">I", 4)
        b[152:154] = crc16(b[:152]).to_bytes(2, "big")

        def wait(m, j):
            for _ in range(300):
                s = m.status(j["id"])
                if s["status"] == "error":
                    raise AssertionError(s["error"])
                if s["status"] == "done":
                    return s
                time.sleep(0.01)
            raise AssertionError("export timed out")

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "data").mkdir()
            (root / "recordings").mkdir()
            (root / "data/catalog.json").write_text("[]")
            (root / "recordings/test.bin").write_bytes(b)
            m = HistoryExport(root)
            try:
                idx = wait(m, m.index("recordings:test.bin"))
                job = wait(m, m.export(idx["id"]))
                with zipfile.ZipFile(m.download(job["id"])) as z:
                    row = list(
                        csv.DictReader(
                            io.StringIO(z.read("samples.csv").decode("utf-8-sig"))
                        )
                    )[0]
                    self.assertEqual(row["life_sensor_fault"], "1")
                    self.assertEqual(row["life_primary_core_fault"], "1")
                    self.assertEqual(row["life_seam_age_ms"], "160")
            finally:
                m.pool.shutdown(wait=True)


if __name__ == "__main__":
    unittest.main()
