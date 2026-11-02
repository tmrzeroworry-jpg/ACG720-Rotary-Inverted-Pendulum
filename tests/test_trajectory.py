import unittest, sys, struct, math, types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Mac上位机"))
from trajectory import *


class Tests(unittest.TestCase):
    def c(self, p, **kw):
        return validate_curve(dict(points=p, **kw))

    def test_curve(self):
        c = self.c([[0, 0], [2, 45], [4, 0]])
        self.assertEqual(value_at(c, 1), 22.5)
        self.assertEqual(value_at(c, 5), 0)

    def test_duplicate(self):
        for p in (
            [[0, 0], [1, 10], [1, 20], [3, 0]],
            [[0, 0], [2, 10], [1, 20], [3, 0]],
        ):
            with self.assertRaises(ValueError):
                self.c(p)

    def test_nonfinite(self):
        for v in (float("nan"), float("inf"), True, "9"):
            with self.assertRaises(ValueError):
                self.c([[0, 0], [1, v], [2, 0]])

    def test_acceleration(self):
        with self.assertRaises(ValueError):
            self.c([[0, 0], [0.1, 60], [2, 0]])

    def test_quantization(self):
        c = self.c([[0, 0], [2, 45], [4, 0]], amax=60)
        self.assertLessEqual(c["amax"], 60)
        with self.assertRaises(ValueError):
            self.c([[0, 0], [1, 60], [2, 0]], amax=60)

    def test_speed_clip(self):
        c = self.c([[0, 0], [4, 100], [8, 0]], vmax=60)
        self.assertEqual(c["points"][1][1], 60)
        self.assertEqual(c["clipped_points"], 1)

    def test_endpoints(self):
        for p in ([[0, 10], [3, 0]], [[0, 0], [3, 10]], [[1, 0], [3, 0]]):
            with self.assertRaises(ValueError):
                self.c(p)

    def test_packet(self):
        b = packet(2, 65535, 0x12345678, 45, 11)
        self.assertEqual(len(b), 24)
        self.assertEqual(crc16(b[:20]), int.from_bytes(b[20:22], "big"))
        self.assertEqual(b[-2:], b"\r\n")
        self.assertEqual(int.from_bytes(b[8:12], "big"), 0x12345678)

    def test_large_curve(self):
        with self.assertRaises(ValueError):
            self.c([[0, 0], [121, 0]])
        with self.assertRaises(ValueError):
            self.c([[i * 0.1, 0] for i in range(17)])

    def engine(self):
        self.now = 10.0
        self.t = Trajectory(lambda: self.now)
        return types.SimpleNamespace(
            mode="live",
            last_receive=10.0,
            latest=dict(
                build="UI02",
                protocol_version=8,
                state=4,
                stream_velocity_q16=0,
                stream_desired_q16=0,
                stream_session=0,
                stream_seq=0,
                stream_flags=1,
                stream_result=0,
                trial=1,
                epoch=0,
                arm_rate=0,
            ),
        )

    def test_handshake_and_stop(self):
        e = self.engine()
        self.t.start(e, "page0001")
        self.assertEqual(self.t.step(e)[4], 0)
        f = e.latest
        f.update(stream_session=self.t.session, stream_flags=33)
        self.now += 0.05
        e.last_receive = self.now
        self.assertEqual(self.t.step(e)[4], 1)
        f.update(stream_flags=35)
        self.now += 0.05
        e.last_receive = self.now
        self.assertEqual(self.t.step(e)[4], 2)
        self.assertEqual(self.t.phase, "running")
        self.t.stop()
        self.assertEqual(self.t.step(e)[4], 3)
        self.assertEqual(self.t.phase, "stopping")
        f.update(stream_seq=self.t.stop_seq, stream_desired_q16=0)
        self.now += 0.05
        e.last_receive = self.now
        self.assertIsNone(self.t.step(e))
        self.assertEqual(self.t.phase, "done")

    def test_unsupported(self):
        e = self.engine()
        e.latest["build"] = "WEXH"
        with self.assertRaises(ValueError):
            self.t.start(e, "page0001")

    def test_owner(self):
        e = self.engine()
        self.t.start(e, "page0001")
        with self.assertRaises(ValueError):
            self.t.pulse("page0002")
        with self.assertRaises(ValueError):
            self.t.update(dict(points=[[0, 0], [2, 0]], client="page0002"))

    def test_no_start_from_poll_or_heartbeat(self):
        e = self.engine()
        self.t.pulse("page0001")
        self.assertIsNone(self.t.step(e))
        self.assertEqual(self.t.phase, "idle")

    def test_timeout(self):
        e = self.engine()
        self.t.start(e, "page0001")
        self.t.step(e)
        self.now += 0.31
        self.assertIsNone(self.t.step(e))
        self.assertEqual(self.t.phase, "error")

    def test_exit_balance(self):
        e = self.engine()
        self.t.start(e, "page0001")
        e.latest["state"] = 2
        self.assertIsNone(self.t.step(e))
        self.assertEqual(self.t.phase, "error")

    def test_stale_editor_revision(self):
        e = self.engine()
        d = dict(points=[[0, 0], [2, 0]], client="page0001", editor_revision=2)
        self.t.update(d)
        with self.assertRaises(ValueError):
            self.t.update(dict(d, editor_revision=1))

    def test_rejected_start_keeps_owner(self):
        e = self.engine()
        self.t.start(e, "page0001")
        with self.assertRaises(ValueError):
            self.t.start(e, "page0002")
        self.assertEqual(self.t.owner, "page0001")

    def test_watchdog_does_not_extend_stop(self):
        e = self.engine()
        self.t.start(e, "page0001")
        self.t.phase = "running"
        self.t.started = self.now
        self.now += 0.6
        e.last_receive = self.now
        e.latest.update(stream_session=self.t.session, stream_flags=35)
        self.t.step(e)
        deadline = self.t.deadline
        self.now += 0.1
        e.last_receive = self.now
        self.t.step(e)
        self.assertEqual(self.t.deadline, deadline)


if __name__ == "__main__":
    unittest.main(verbosity=2)
