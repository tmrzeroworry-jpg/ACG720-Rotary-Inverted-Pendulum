import json, sys, tempfile, threading, time, unittest, urllib.request, urllib.error
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Mac上位机"))
import server


class HTTPControls(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        p = Path(self.tmp.name)
        (p / "data").mkdir()
        (p / "data/catalog.json").write_text("[]")
        self.e = server.Engine(p)
        self.e.mode = "live"
        self.e.latest = dict(
            build="UI02",
            protocol_version=8,
            state=1,
            life_phase=0,
            life_down_valid=True,
            remote_flags=12,
            remote_session=0,
            remote_mode=0,
            remote_cfg=0,
        )
        self.http = server.ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
        self.http.engine = self.e
        self.worker = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.worker.start()
        self.url = "http://127.0.0.1:" + str(self.http.server_port)

    def tearDown(self):
        self.http.shutdown()
        self.http.server_close()
        self.worker.join(1)
        self.e.stop()
        self.e.history.pool.shutdown(wait=True)
        self.tmp.cleanup()

    def post(self, path, data, origin=None):
        self.e.last_receive = time.monotonic()
        h = {"Content-Type": "application/json"}
        if origin:
            h["Origin"] = origin
        req = urllib.request.Request(
            self.url + path, data=json.dumps(data).encode(), headers=h, method="POST"
        )
        try:
            with urllib.request.urlopen(req) as r:
                return r.status, json.load(r)
        except urllib.error.HTTPError as r:
            return r.code, json.load(r)

    def own(self):
        self.assertEqual(
            self.post("/api/control/claim", {"client": "browser01"})[0], 200
        )
        self.e.control.step(self.e)
        self.e.latest.update(
            remote_session=self.e.control.session,
            remote_flags=13,
            remote_seq=self.e.control.seq,
            remote_op=0,
        )
        self.e.control.step(self.e)

    def test_api_no_owner_no_crosssite_and_stale_source(self):
        self.assertEqual(
            self.post(
                "/api/control/action", {"client": "browser01", "action": "start"}
            )[0],
            400,
        )
        self.assertEqual(
            self.post(
                "/api/control/claim", {"client": "browser01"}, "http://external.invalid"
            )[0],
            403,
        )
        self.e.mode = "replay"
        self.assertEqual(
            self.post("/api/control/claim", {"client": "browser01"})[0], 400
        )

    def test_one_pending_operation_blocks_other_modes_and_stop_wins(self):
        self.own()
        self.assertEqual(
            self.post(
                "/api/control/action",
                {"client": "browser01", "action": "start", "value": 0},
            )[0],
            200,
        )
        self.assertEqual(
            self.post(
                "/api/control/action",
                {"client": "browser01", "action": "start", "value": 1},
            )[0],
            400,
        )
        self.e.latest["state"] = 4
        self.assertEqual(
            self.post("/api/flip/start", {"client": "browser01", "direction": 1})[0],
            400,
        )
        self.assertEqual(
            self.post("/api/return/start", {"client": "browser01"})[0], 400
        )
        self.assertEqual(
            self.post("/api/trajectory/start", {"client": "browser01"})[0], 400
        )
        self.assertEqual(
            self.post("/api/control/stop", {"client": "another-page"})[0], 200
        )
        self.assertIsNone(self.e.control.pending)
        self.assertEqual(self.e.control.step(self.e)[4], 7)

    def test_physical_stop_and_sensor_flags_refuse_claim(self):
        for bits in (0, 4, 8):
            self.e.latest["remote_flags"] = bits
            self.assertEqual(
                self.post("/api/control/claim", {"client": "browser01"})[0], 400
            )


if __name__ == "__main__":
    unittest.main()
