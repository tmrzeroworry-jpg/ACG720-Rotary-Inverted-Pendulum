import sys, types, unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Mac上位机"))
from flip import Flip, flip_packet
from protocol_vendor import crc16


class Tests(unittest.TestCase):
    def engine(self):
        self.now = 10.0
        self.flip = Flip(lambda: self.now)
        return types.SimpleNamespace(
            mode="live",
            last_receive=self.now,
            trajectory=types.SimpleNamespace(phase="idle"),
            latest=dict(
                build="UI02",
                protocol_version=8,
                state=4,
                stream_velocity_q16=0,
                flip_id=1,
                flip_phase=0,
                flip_result=0,
                stream_session=0,
                stream_flags=1,
                stream_seq=0,
            ),
        )

    def test_protocol(self):
        for direction in (-1, 1):
            b = flip_packet(4, 65535, 123, direction)
            self.assertEqual(len(b), 24)
            self.assertEqual(int.from_bytes(b[12:16], "big", signed=True), direction)
            self.assertEqual(crc16(b[:20]), int.from_bytes(b[20:22], "big"))

    def test_handshake_single_start_keepalive_and_finish(self):
        e = self.engine()
        m = self.flip
        m.start(e, "client01", 1)
        self.assertEqual(m.step(e)[4], 0)
        e.latest.update(stream_session=m.session, stream_flags=33)
        self.assertEqual(m.step(e)[4], 4)
        self.assertIsNone(m.step(e))
        e.latest.update(flip_id=2, flip_phase=1)
        self.assertEqual(m.step(e)[4], 5)
        self.now += 0.11
        e.last_receive = self.now
        m.pulse("client01")
        self.assertEqual(m.step(e)[4], 5)
        e.latest["flip_phase"] = 5
        self.assertIsNone(m.step(e))
        self.assertEqual(m.phase, "done")

    def test_reject_curve_running(self):
        e = self.engine()
        e.trajectory.phase = "running"
        with self.assertRaises(ValueError):
            self.flip.start(e, "client01", 1)

    def test_unsupported_or_not_balanced(self):
        e = self.engine()
        e.latest["build"] = "TR01"
        with self.assertRaises(ValueError):
            self.flip.start(e, "client01", 1)
        e.latest["build"] = "TR04"
        e.latest["state"] = 1
        with self.assertRaises(ValueError):
            self.flip.start(e, "client01", 1)

    def test_no_auto_start(self):
        e = self.engine()
        self.assertIsNone(self.flip.step(e))
        self.assertEqual(self.flip.phase, "idle")

    def test_browser_lease_and_cancel(self):
        e = self.engine()
        m = self.flip
        m.start(e, "client01", 1)
        with self.assertRaises(ValueError):
            m.pulse("client02")
        e.latest.update(stream_session=m.session, stream_flags=33)
        self.assertEqual(m.step(e)[4], 4)
        e.latest.update(flip_id=2, flip_phase=2)
        m.step(e)
        self.now += 0.6
        e.last_receive = self.now
        self.assertEqual(m.step(e)[4], 6)
        self.assertIsNone(m.step(e))

    def test_reverse_not_success(self):
        e = self.engine()
        m = self.flip
        m.start(e, "client01", -1)
        e.latest.update(stream_session=m.session, stream_flags=33)
        m.step(e)
        e.latest.update(flip_id=2, flip_phase=2)
        m.step(e)
        e.latest.update(flip_phase=6, flip_result=3)
        self.assertIsNone(m.step(e))
        self.assertEqual(m.phase, "error")
        self.assertIn("反向", m.message)


if __name__ == "__main__":
    unittest.main(verbosity=2)
