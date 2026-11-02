"""WEXH tests use bytes emitted by the unchanged Verilog serializer, no hardware."""

import csv, io, json, sys, tempfile, time, unittest, zipfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from telemetry import Decoder, normalize
from protocol_vendor import crc16
from history_export import HistoryExport

APP = Path(__file__).resolve().parents[1]
VECTOR = (APP / "tests/wexh_uart_vectors.bin").read_bytes()


def frame(profile=5, build=b"WEXH"):
    b = bytearray(VECTOR[:108])
    b[4:8] = build
    b[20] = profile
    b[104:106] = crc16(b[:104]).to_bytes(2, "big")
    return bytes(b)


def wait(m, j):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        s = m.status(j["id"])
        if s["status"] == "done":
            return s
        if s["status"] == "error":
            raise AssertionError(s["error"])
        time.sleep(0.01)
    raise AssertionError("timeout")


class WEXHTests(unittest.TestCase):
    def test_original_serializer_units_profile_and_counter_wrap(self):
        d = Decoder()
        rows = []
        for b in VECTOR:
            rows.extend(d.feed(bytes([b])))
        self.assertEqual(d.bad_crc, 0)
        self.assertEqual(len(rows), 6)
        self.assertEqual(
            [r["balance_ms"] for r in rows], [65530, 65540, 65550, 65560, 65570, 65580]
        )
        v = normalize(rows[0])
        self.assertEqual(v["build"], "WEXH")
        self.assertTrue(v["pose_valid"])
        self.assertEqual(v["counts_per_rev"], 1040)
        self.assertEqual(v["arm_unwrapped"], -360)
        self.assertEqual(v["arm_rate"], 180)
        self.assertIsNone(v["duration"])
        self.assertEqual(v["route"], "WT01 · PD")
        self.assertEqual(v["pole_filter"], "1€ 快速")
        self.assertTrue(v["balance_lower_bound"])

    def test_all_36_profile_combinations_match_WEXF(self):
        for base in range(4):
            for pole in range(3):
                for arm in range(3):
                    profile = base * 16 + pole * 4 + arm + 1
                    h = normalize(Decoder().feed(frame(profile))[0])
                    f = normalize(Decoder().feed(frame(profile, b"WEXF"))[0])
                    self.assertEqual(
                        h["pole_filter"], ["原滤波", "1€ 快速", "1€ 平滑"][pole]
                    )
                    self.assertEqual(
                        h["arm_filter"], ["原滤波", "1€ 快速", "1€ 平滑"][arm]
                    )
                    for key in h:
                        if key not in ("build", "warning"):
                            self.assertEqual(h[key], f[key], (profile, key))
        d = Decoder()
        self.assertEqual(d.feed(frame(4)), [])

    def test_windows_export_preserves_WEXH_raw_and_sensor_values(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "recordings").mkdir()
            p = root / "recordings/h.bin"
            p.write_bytes(VECTOR)
            m = HistoryExport(root)
            try:
                index = wait(m, m.index("recordings:h.bin"))
                self.assertEqual(index["result"]["trials"][0]["build"], "WEXH")
                job = wait(m, m.export(index["id"]))
                with zipfile.ZipFile(m.download(job["id"])) as z:
                    data = z.read("samples.csv")
                    self.assertTrue(data.startswith(b"\xef\xbb\xbf"))
                    self.assertIn(b"\r\n", data)
                    rows = list(csv.DictReader(io.StringIO(data.decode("utf-8-sig"))))
                    self.assertEqual(len(rows), 6)
                    r = rows[0]
                    self.assertEqual(r["firmware"], "WEXH")
                    self.assertEqual(r["encoder_counts_per_rev"], "1040")
                    self.assertEqual(r["encoder_counts_total"], "-1040")
                    self.assertEqual(r["encoder_counts_mod"], "0")
                    self.assertAlmostEqual(
                        float(r["potentiometer_voltage_v"]), 6223 * 0.000125
                    )
                    self.assertAlmostEqual(float(r["arm_velocity_deg_s"]), 180)
                    self.assertTrue(0 <= float(r["pole_display_deg"]) <= 180)
                    self.assertEqual(z.read("raw_uart.bin"), VECTOR)
                    self.assertEqual(p.read_bytes(), VECTOR)
            finally:
                m.pool.shutdown(wait=True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
