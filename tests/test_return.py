import sys, types, unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Mac上位机"))
from return_motion import ReturnMotion as Flip
from protocol_vendor import crc16


class Tests(unittest.TestCase):
    def engine(self):
        self.now = 10.0
        self.flip = Flip(lambda: self.now)
        return types.SimpleNamespace(
            mode="live",
            last_receive=self.now,
            trajectory=types.SimpleNamespace(phase="idle"),
            flip=types.SimpleNamespace(phase="idle"),
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
                life_id=1,
                life_mode=0,
                life_phase=0,
                life_result=0,
                life_down_valid=True,
            ),
        )

    def test_no_auto_start_and_replay_refusal(self):
        e = self.engine()
        m = self.flip
        self.assertIsNone(m.step(e))
        self.assertEqual(m.phase, "idle")
        e.mode = "replay"
        with self.assertRaises(ValueError):
            m.start(e, "client01")

    def test_learned_reference_gate(self):
        e = self.engine()
        e.latest["life_down_valid"] = False
        with self.assertRaises(ValueError):
            self.flip.start(e, "client01")

    def test_handshake_single_start_and_completion(self):
        e = self.engine()
        m = self.flip
        m.start(e, "client01")
        self.assertEqual(m.step(e)[4], 0)
        e.latest.update(stream_session=m.session, stream_flags=33)
        self.assertEqual(m.step(e)[4], 7)
        self.assertIsNone(m.step(e))
        e.latest.update(life_id=2, life_phase=4, life_mode=1)
        self.assertEqual(m.step(e)[4], 8)
        self.now += 0.11
        e.last_receive = self.now
        m.pulse("client01")
        self.assertEqual(m.step(e)[4], 8)
        e.latest["life_phase"] = 10
        self.assertIsNone(m.step(e))
        self.assertEqual(m.phase, "done")

    def test_browser_timeout_cancel_and_sensor_loss(self):
        e = self.engine()
        m = self.flip
        m.start(e, "client01")
        e.latest.update(stream_session=m.session, stream_flags=33)
        m.step(e)
        e.latest.update(life_id=2, life_phase=6, life_mode=1)
        m.step(e)
        self.now += 0.6
        e.last_receive = self.now
        self.assertEqual(m.step(e)[4], 9)
        self.assertIsNone(m.step(e))
        e = self.engine()
        m = self.flip
        m.start(e, "client01")
        self.now += 0.31
        self.assertIsNone(m.step(e))
        self.assertEqual(m.phase, "error")

    def test_no_duplicate_start_on_lost_ack(self):
        e = self.engine()
        m = self.flip
        m.start(e, "client01")
        e.latest.update(stream_session=m.session, stream_flags=33)
        self.assertEqual(m.step(e)[4], 7)
        for _ in range(12):
            self.now += 0.1
            e.last_receive = self.now
            m.pulse("client01")
            self.assertIsNone(m.step(e))
        self.assertEqual(m.phase, "error")
        self.assertEqual(len([x for x in m.events if x.get("op") == 7]), 1)

    def test_curve_waits_until_stopped(self):
        e = self.engine()
        m = self.flip
        e.trajectory.phase = "running"
        e.trajectory.stop = lambda: setattr(e.trajectory, "phase", "stopping")
        m.start(e, "client01")
        self.assertIsNone(m.step(e))
        self.assertEqual(m.phase, "waiting_curve")
        e.trajectory.phase = "done"
        e.latest.update(stream_velocity_q16=0, stream_desired_q16=0)
        self.assertIsNone(m.step(e))
        self.assertEqual(m.phase, "hello")


if __name__ == "__main__":
    unittest.main(verbosity=2)
