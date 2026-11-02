import sys, unittest, tempfile, shutil, json, os, pty, select, time, hashlib, threading, urllib.request, urllib.error
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import server
from telemetry import Decoder, normalize, parse_record
from protocol_vendor import crc16

ROOT = Path(__file__).resolve().parents[1]
SAMPLE = (ROOT / "data/WE20_trial3.bin").read_bytes()


class ProtocolTests(unittest.TestCase):
    def test_all_fragments_recover_corruption(self):
        good = SAMPLE[:108]
        bad = bytearray(good)
        bad[40] ^= 8
        d = Decoder()
        out = []
        data = b"noise" + good + bytes(bad) + good
        for b in data:
            out += d.feed(bytes([b]))
        self.assertEqual(len(out), 2)
        self.assertEqual(d.bad_crc, 1)
        self.assertEqual(d.buffer, b"")

    def test_reference_and_target_are_not_fabricated(self):
        f, _, _ = parse_record(SAMPLE)
        v = normalize(f[0])
        self.assertTrue(v["reference_ok"])
        self.assertIsNotNone(v["target"])
        x = dict(f[0], build_id="IRDG", angle_up=0, angle_down=0, state=8)
        v = normalize(x)
        self.assertIsNone(v["pole"])
        self.assertFalse(v["reference_ok"])
        self.assertIn("红外只读", v["warning"])
        x = dict(f[0])
        x.pop("arm_target_counts")
        v = normalize(x)
        self.assertIsNone(v["arm_error"])
        self.assertIsNone(v["target"])

    def test_reboot_not_billions_missing_and_16bit_negative(self):
        d = Decoder()
        a = bytearray(SAMPLE[:108])
        a[8:12] = (100).to_bytes(4, "big")
        a[12:16] = (10000).to_bytes(4, "big")
        a[26:28] = (-200).to_bytes(2, "big", signed=True)
        a[104:106] = crc16(a[:104]).to_bytes(2, "big")
        self.assertEqual(d.feed(a)[0]["error"], -200)
        a[8:12] = (1).to_bytes(4, "big")
        a[12:16] = (10).to_bytes(4, "big")
        a[104:106] = crc16(a[:104]).to_bytes(2, "big")
        self.assertEqual(d.feed(a)[0]["epoch"], 1)
        self.assertEqual(d.missing, 0)

    def test_actual_four_complete_trials(self):
        for n in range(1, 5):
            f, t, q = parse_record((ROOT / f"data/WE20_trial{n}.bin").read_bytes())
            self.assertEqual(q["bad_crc"], 0)
            self.assertTrue(any(x["fault"] == 5 for x in f))
            self.assertTrue(any(x["balance_ms"] >= 20000 for x in f))
            self.assertTrue(all(b >= a for a, b in zip(t, t[1:])))


class EngineTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        r = Path(self.temp.name)
        shutil.copytree(ROOT / "data", r / "data")
        (r / "recordings").mkdir()
        self.e = server.Engine(r)

    def tearDown(self):
        self.e.stop()
        self.temp.cleanup()

    def test_replay_seek_playback_metrics(self):
        e = self.e
        e.replay_load("WE20_trial3.bin")
        e.replay_action("seek", position=e.times[-1])
        s = e.status()
        row = next(r for r in s["trials"] if r["trial"] == 3)
        self.assertGreaterEqual(row["balance"], 20)
        self.assertIsNotNone(row["rms"])
        self.assertIsNotNone(row["arm_range"])
        self.assertEqual(row["stage"], "正常完成")
        self.assertIn("trial", e.csv().decode("utf-8-sig"))
        g = s["generation"]
        e.replay_action("seek", position=0)
        self.assertGreater(e.status()["generation"], g)
        e.replay_action("play")
        time.sleep(0.08)
        e.replay_action("pause")
        self.assertGreater(e.position, 0)
        self.assertFalse(e.playing)

    def test_import_validation(self):
        with self.assertRaises(ValueError):
            self.e.import_record(b"junk", "bad.bin")
        name = self.e.import_record(SAMPLE, "<script>.bin")
        self.assertTrue(name.startswith("import_"))
        self.assertEqual(self.e.catalog[-1]["label"], "<script>.bin")

    def test_serial_receive_record_disconnect_no_transmit(self):
        master, slave = pty.openpty()
        port = os.ttyname(slave)
        real = server.ports
        server.ports = lambda: [port]
        try:
            self.e.connect(port)
            # Check competing collector is rejected, preserving existing reader.
            e2 = server.Engine(self.e.root)
            with self.assertRaises(OSError):
                e2.connect(port)
            payload = SAMPLE[: 108 * 12]
            os.write(master, payload)
            deadline = time.monotonic() + 2
            while (
                self.e.status()["quality"].get("valid", 0) < 12
                and time.monotonic() < deadline
            ):
                time.sleep(0.02)
            status = self.e.status()
            self.assertEqual(status["quality"]["valid"], 12)
            self.assertFalse(status["stale"])
            # USB may deliver multiple frames in one read; preserve device sample timing.
            self.assertAlmostEqual(
                status["frames"][-1]["t"] - status["frames"][0]["t"], 0.11, places=5
            )
            self.assertFalse(
                select.select([master], [], [], 0.05)[0],
                "host unexpectedly transmitted or echoed",
            )
            time.sleep(0.55)
            self.assertTrue(self.e.status()["stale"])
            os.close(master)
            master = None
            time.sleep(0.3)
            self.assertTrue(self.e.error)
            self.e.stop()
            logs = list((self.e.root / "recordings").glob("*.bin"))
            self.assertEqual(len(logs), 1)
            self.assertEqual(logs[0].read_bytes(), payload)
            meta = json.loads(logs[0].with_suffix(".session.json").read_text())
            self.assertFalse(meta["transmits_data"])
            self.assertEqual(meta["sha256"], hashlib.sha256(payload).hexdigest())
        finally:
            server.ports = real
            if master is not None:
                os.close(master)
            os.close(slave)

    def test_invalid_port_does_not_stop_playback(self):
        self.e.replay_load("WE20_trial1.bin")
        self.e.replay_action("play")
        with self.assertRaises(ValueError):
            self.e.connect("/dev/not-a-device")
        self.assertTrue(self.e.playing)


class HTTPTests(unittest.TestCase):
    setUp = EngineTests.setUp
    tearDown = EngineTests.tearDown

    def test_http_security_and_endpoints(self):
        self.e.replay_load("WE20_trial3.bin")
        srv = server.ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
        srv.engine = self.e
        t = threading.Thread(target=srv.serve_forever, daemon=True)
        t.start()
        base = f"http://127.0.0.1:{srv.server_port}"
        try:
            with urllib.request.urlopen(base + "/api/status") as r:
                self.assertEqual(json.load(r)["mode"], "replay")
            with self.assertRaises(urllib.error.HTTPError) as ctx:
                urllib.request.urlopen(
                    urllib.request.Request(
                        base + "/api/disconnect",
                        data=b"{}",
                        headers={
                            "Content-Type": "application/json",
                            "Origin": "https://example.com",
                        },
                    )
                )
            self.assertEqual(ctx.exception.code, 403)
            self.assertEqual(self.e.mode, "replay")
            with self.assertRaises(urllib.error.HTTPError):
                urllib.request.urlopen(base + "/../../server.py")
            with self.assertRaises(urllib.error.HTTPError):
                urllib.request.urlopen(
                    urllib.request.Request(
                        base + "/api/status", headers={"Host": "evil.test"}
                    )
                )
            with urllib.request.urlopen(
                urllib.request.Request(
                    base + "/api/replay/action",
                    data=json.dumps(
                        dict(action="seek", position=self.e.times[-1])
                    ).encode(),
                    headers={"Content-Type": "application/json"},
                )
            ) as r:
                self.assertTrue(json.load(r)["ok"])
            with urllib.request.urlopen(base + "/api/export") as r:
                self.assertIn("trial", r.read().decode("utf-8-sig"))
        finally:
            srv.shutdown()
            srv.server_close()
            t.join()


if __name__ == "__main__":
    unittest.main(verbosity=2)
