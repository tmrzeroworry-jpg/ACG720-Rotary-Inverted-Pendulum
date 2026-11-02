import sys, struct, unittest, types, tempfile, time, csv, io, zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Mac上位机"))
from controls import Controls, packet
from protocol_vendor import crc16, decode_frame
from telemetry import Decoder, normalize


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.c = Controls(lambda: self.now)

        def dummy():
            o = types.SimpleNamespace(phase="idle")
            o.disconnect = lambda: setattr(o, "phase", "idle")
            return o

        self.e = types.SimpleNamespace(
            mode="live",
            last_receive=self.now,
            latest=dict(
                build="UI02",
                protocol_version=8,
                state=1,
                life_phase=0,
                life_down_valid=True,
                remote_flags=12,
                remote_session=0,
                remote_seq=0,
                remote_op=0,
                remote_result=0,
                remote_mode=0,
                remote_cfg=0,
            ),
            flip=dummy(),
            return_motion=dummy(),
            trajectory=dummy(),
        )

    def owned(self):
        c, e = self.c, self.e
        c.claim(e, "client001")
        b = c.step(e)
        self.assertEqual(b[4], 0)
        e.latest.update(
            remote_session=c.session, remote_seq=c.seq, remote_op=0, remote_flags=13
        )
        c.step(e)
        self.assertEqual(c.phase, "active")

    def test_packet_and_no_automatic_control(self):
        self.assertIsNone(self.c.step(self.e))
        b = packet(5, 123, 0x12345678, 7)
        self.assertEqual(len(b), 24)
        self.assertEqual(b[:4], b"\xaa\xc4\x01\x18")
        self.assertEqual(crc16(b[:20]), int.from_bytes(b[20:22], "big"))

    def test_replay_and_old_firmware_refused(self):
        self.e.mode = "replay"
        with self.assertRaises(ValueError):
            self.c.claim(self.e, "client001")
        self.e.mode = "live"
        self.e.latest["build"] = "TR06"
        with self.assertRaises(ValueError):
            self.c.claim(self.e, "client001")

    def test_explicit_claim_single_start_and_ack(self):
        self.owned()
        self.c.action(self.e, "client001", "start", 0)
        b = self.c.step(self.e)
        self.assertEqual(b[4], 2)
        self.assertIsNone(self.c.step(self.e))
        self.e.latest.update(
            remote_seq=self.c.seq, remote_op=2, remote_result=0, state=9, life_phase=1
        )
        self.c.step(self.e)
        self.assertIsNone(self.c.waiting)

    def test_second_page_and_no_start_queue(self):
        self.owned()
        with self.assertRaises(ValueError):
            self.c.action(self.e, "client002", "start")
        self.e.latest["state"] = 9
        with self.assertRaises(ValueError):
            self.c.action(self.e, "client001", "start")

    def test_atomic_arguments_validation(self):
        self.owned()
        self.e.latest["state"] = 4
        for a, v in [
            ("position", 8),
            ("sine", -1),
            ("spin", 4),
            ("hold", 1),
            ("sine", True),
            ("sine", 1.0),
            ("bad", 0),
        ]:
            with self.assertRaises(ValueError, msg=(a, v)):
                self.c.action(self.e, "client001", a, v)

    def test_life_flip_curve_mutual_exclusion(self):
        self.owned()
        self.e.latest["state"] = 4
        for name, phase in [
            ("flip", "running"),
            ("return_motion", "pending"),
            ("trajectory", "running"),
        ]:
            obj = getattr(self.e, name)
            obj.phase = phase
            with self.assertRaises(ValueError):
                self.c.action(self.e, "client001", "position", 0)
            obj.phase = "idle"
        self.e.latest["life_phase"] = 6
        with self.assertRaises(ValueError):
            self.c.action(self.e, "client001", "position", 0)

    def test_missing_ack_stops_never_retries_start(self):
        self.owned()
        self.c.action(self.e, "client001", "start")
        self.c.step(self.e)
        self.now += 0.26
        self.e.last_receive = self.now
        self.c.pulse("client001")
        b = self.c.step(self.e)
        self.assertEqual(b[4], 7)
        self.assertEqual(sum(x.get("op") == 2 for x in self.c.events), 1)

    def test_page_loss_stops_and_cancels_all_writers(self):
        self.owned()
        self.e.flip.phase = "running"
        self.e.trajectory.phase = "running"
        self.e.return_motion.phase = "running"
        self.now += 0.61
        self.e.last_receive = self.now
        b = self.c.step(self.e)
        self.assertEqual(b[4], 7)
        self.assertEqual(self.e.flip.phase, "idle")
        self.assertEqual(self.e.trajectory.phase, "idle")

    def test_stop_without_old_ownership(self):
        self.c.stop(self.e)
        b = self.c.step(self.e)
        self.assertEqual(b[4], 7)
        self.assertIsNone(self.c.step(self.e))
        self.e.latest.update(
            remote_session=self.c.session, remote_seq=self.c.seq, remote_op=7
        )
        self.c.step(self.e)
        self.assertEqual(self.c.phase, "idle")

    def test_s0_revokes_owner_and_no_resume(self):
        self.owned()
        self.e.latest.update(remote_flags=12, remote_result=4)
        self.c.step(self.e)
        self.assertEqual(self.c.phase, "error")
        self.assertIsNone(self.c.step(self.e))

    def test_special_requires_hold_after_task(self):
        self.owned()
        self.e.latest.update(state=4, remote_mode=2, remote_cfg=8)
        with self.assertRaises(ValueError):
            self.c.special(self.e, "client001")
        self.e.latest.update(remote_mode=1, remote_cfg=128)
        self.c.special(self.e, "client001")

    def test_curve_accepts_ui01_v8_and_keeps_owner_gate(self):
        from trajectory import Trajectory

        self.owned()
        self.e.latest.update(
            state=4, epoch=0, trial=1, stream_flags=1, stream_velocity_q16=0
        )
        m = Trajectory(lambda: self.now)
        m.start(self.e, "client001")
        self.assertEqual(m.step(self.e)[4], 0)

    def test_only_validated_start_and_data_freshness(self):
        self.owned()
        self.e.latest["life_down_valid"] = False
        with self.assertRaises(ValueError):
            self.c.action(self.e, "client001", "start", 0)
        self.c.action(self.e, "client001", "start", 1)
        self.now += 0.31
        self.assertIsNone(self.c.step(self.e))
        self.assertEqual(self.c.phase, "error")


class V8Tests(unittest.TestCase):
    def test_actual_serializer_fields_and_chunking(self):
        data = (ROOT / "tests/remote_uart_vectors.bin").read_bytes()
        self.assertEqual(len(data), 1032)
        d = Decoder()
        rows = []
        for i in range(0, len(data), 13):
            rows += d.feed(data[i : i + 13])
        self.assertEqual(len(rows), 6)
        self.assertEqual(d.quality()["bad_crc"], 0)
        for f in rows:
            self.assertEqual(
                (
                    f["build_id"],
                    f["protocol_version"],
                    f["remote_session"],
                    f["remote_seq"],
                    f["remote_op"],
                    f["remote_mode"],
                    f["remote_cfg"],
                    f["remote_age_ms"],
                ),
                ("UI02", 8, 0x89ABCDEF, 234, 5, 3, 112, 97),
            )
            self.assertEqual(normalize(f)["counts_per_rev"], 1040)
            self.assertTrue(f["life_down_valid"])

    def test_v8_export_preserves_encoder_adc_and_commands(self):
        from history_export import HistoryExport

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "data").mkdir()
            (root / "recordings").mkdir()
            (root / "data/catalog.json").write_text("[]")
            data = (ROOT / "tests/remote_uart_vectors.bin").read_bytes()
            (root / "recordings/v8.bin").write_bytes(data)
            m = HistoryExport(root)

            def wait(j):
                for _ in range(400):
                    s = m.status(j["id"])
                    if s["status"] == "done":
                        return s
                    if s["status"] == "error":
                        self.fail(s["error"])
                    time.sleep(0.01)
                self.fail("export timeout")

            try:
                j = wait(m.index("recordings:v8.bin"))
                out = wait(m.export(j["id"]))
                with zipfile.ZipFile(m.download(out["id"])) as z:
                    rows = list(
                        csv.DictReader(
                            io.StringIO(z.read("samples.csv").decode("utf-8-sig"))
                        )
                    )
                    self.assertEqual(len(rows), 6)
                    self.assertEqual(rows[0]["remote_op"], "5")
                    self.assertEqual(rows[0]["encoder_counts_total"], "-1040")
                    self.assertEqual(rows[0]["potentiometer_adc_raw"], "6223")
                    self.assertEqual(z.read("raw_uart.bin"), data)
            finally:
                m.pool.shutdown(wait=True)


if __name__ == "__main__":
    unittest.main()
