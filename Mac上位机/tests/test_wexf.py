"""WEXF adapter checks; all inputs are protocol simulation or saved records."""

import json, sys, unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from telemetry import Decoder, normalize, parse_record, TrialMetrics
from protocol_vendor import crc16

ROOT = Path(__file__).resolve().parents[1]


def edited(changes):
    b = bytearray((ROOT / "tests/wexf_uart_vectors.bin").read_bytes()[:108])
    for (offset, size), value in changes.items():
        b[offset : offset + size] = value.to_bytes(size, "big", signed=value < 0)
    b[104:106] = crc16(b[:104]).to_bytes(2, "big")
    return bytes(b)


class WEXFTests(unittest.TestCase):
    def test_original_verilog_serializer_and_1040_units(self):
        rows, t, q = parse_record((ROOT / "tests/wexf_uart_vectors.bin").read_bytes())
        self.assertEqual(q["bad_crc"], 0)
        self.assertEqual(len(rows), 6)
        f = rows[0]
        self.assertEqual(f["build_id"], "WEXF")
        self.assertEqual(f["arm_position"], -1040)
        self.assertEqual(f["arm_target_counts"], -910)
        self.assertEqual(f["command"], -85)
        v = normalize(f)
        self.assertTrue(v["known"])
        self.assertEqual(v["counts_per_rev"], 1040)
        self.assertEqual(v["arm_unwrapped"], -360)
        self.assertEqual(v["arm_error"], -45)
        self.assertEqual(v["target_unwrapped"], -315)
        self.assertIsNone(v["duration"])
        self.assertEqual(v["pole_filter"], "1€ 快速")
        self.assertEqual(v["arm_filter"], "原滤波")
        self.assertAlmostEqual(v["pole"], -4 * 360 / 26400)

    def test_counter_wrap_trial_boundary_and_long_gap_are_lower_bound(self):
        d = Decoder()
        rows = d.feed((ROOT / "tests/wexf_uart_vectors.bin").read_bytes())
        self.assertEqual(
            [f["balance_ms"] for f in rows], [65530, 65540, 65550, 65560, 65570, 65580]
        )
        self.assertTrue(all(normalize(f)["balance_lower_bound"] for f in rows))
        reset = edited({(8, 4): 6, (12, 4): 70, (16, 4): 2, (74, 2): 100})
        self.assertEqual(d.feed(reset)[0]["balance_ms"], 100)
        # Joining after multiple unseen wraps cannot prove a complete elapsed time.
        v = normalize(Decoder().feed(edited({(74, 2): 50}))[0])
        self.assertEqual(v["balance"], 0.05)
        self.assertTrue(v["balance_lower_bound"])

    def test_fragment_crc_fault_sensor_and_restart_semantics(self):
        b = (ROOT / "tests/wexf_uart_vectors.bin").read_bytes()[:108]
        d = Decoder()
        out = []
        for v in b:
            out += d.feed(bytes([v]))
        self.assertEqual(len(out), 1)
        bad = bytearray(b)
        bad[40] ^= 8
        d = Decoder()
        out = d.feed(bytes(bad) + b)
        self.assertEqual(d.bad_crc, 1)
        self.assertEqual(len(out), 1)
        v = normalize(Decoder().feed(edited({(21, 1): 2, (74, 2): 100}))[0])
        self.assertEqual(v["stage"], "自动重起摆")
        v = normalize(Decoder().feed(edited({(23, 1): 0}))[0])
        self.assertIsNone(v["pole"])
        self.assertIsNone(v["arm"])
        v = normalize(Decoder().feed(edited({(22, 1): 4, (21, 1): 5}))[0])
        self.assertEqual(v["fault"], 4)
        self.assertNotEqual(v["stage"], "正常完成")

    def test_old_real_WE20_keeps_legacy_calibration_and_limit(self):
        f, _, _ = parse_record((ROOT / "data/WE20_trial1.bin").read_bytes())
        v = normalize(f[0])
        self.assertEqual(v["counts_per_rev"], 1058)
        self.assertEqual(v["duration"], 20)
        self.assertFalse(v["balance_lower_bound"])


if __name__ == "__main__":
    unittest.main()
