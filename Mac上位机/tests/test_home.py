import csv, io, json, sys, tempfile, time, unittest, zipfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from telemetry import Decoder, normalize
from protocol_vendor import crc16
from history_export import HistoryExport

APP = Path(__file__).resolve().parents[1]
DATA = (APP / "tests/home_uart_vectors.bin").read_bytes()


class HomeTests(unittest.TestCase):
    def test_actual_verilog_new_snapshot_signed_home_valid_and_100Hz(self):
        d = Decoder()
        rows = []
        for b in DATA:
            rows += d.feed(bytes([b]))
        self.assertEqual(len(rows), 6)
        self.assertEqual(d.bad_crc, 0)
        self.assertEqual(d.missing, 0)
        self.assertEqual([r["seq"] for r in rows], list(range(6)))
        self.assertEqual([r["arm_home_counts"] for r in rows], [-286] * 6)
        self.assertFalse(rows[0]["arm_home_valid"])
        self.assertTrue(all(r["arm_home_valid"] for r in rows[1:]))
        self.assertEqual(
            [r["arm_target_counts"] for r in rows], [-910 + i * 130 for i in range(6)]
        )
        for a, b in zip(rows, rows[1:]):
            self.assertEqual(b["uptime_ms"] - a["uptime_ms"], 10)
        self.assertEqual(normalize(rows[1])["arm_home_counts"], -286)

    def test_crc_resync_legacy_and_live_snapshot_independence(self):
        bad = bytearray(DATA[:113])
        bad[105] ^= 1
        old = (APP / "tests/wexh_uart_vectors.bin").read_bytes()[:108]
        d = Decoder()
        rows = d.feed(b"junk" + bad + old + DATA)
        self.assertEqual(d.bad_crc, 1)
        self.assertEqual(len(rows), 7)
        self.assertEqual(rows[0]["protocol_version"], 3)
        self.assertNotIn("arm_home_counts", rows[0])
        self.assertEqual(rows[1]["protocol_version"], 4)

    def test_export_keeps_raw_counts_and_adds_separate_relative_columns(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "recordings").mkdir()
            (root / "recordings/h.bin").write_bytes(DATA)
            m = HistoryExport(root)

            def wait(job):
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    s = m.status(job["id"])
                    if s["status"] == "done":
                        return s
                    if s["status"] == "error":
                        raise AssertionError(s["error"])
                    time.sleep(0.01)
                raise AssertionError("timeout")

            try:
                index = wait(m.index("recordings:h.bin"))
                done = wait(m.export(index["id"]))
                with zipfile.ZipFile(m.download(done["id"])) as z:
                    rows = list(
                        csv.DictReader(
                            io.StringIO(z.read("samples.csv").decode("utf-8-sig"))
                        )
                    )
                    self.assertEqual(rows[0]["encoder_relative_counts_total"], "")
                    r = rows[1]
                    self.assertEqual(r["encoder_counts_total"], "-1040")
                    self.assertEqual(r["encoder_counts_mod"], "0")
                    self.assertEqual(r["arm_home_counts"], "-286")
                    self.assertEqual(r["encoder_relative_counts_total"], "-754")
                    self.assertEqual(r["encoder_relative_counts_mod"], "286")
                    self.assertEqual(z.read("raw_uart.bin"), DATA)
            finally:
                m.pool.shutdown(wait=True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
