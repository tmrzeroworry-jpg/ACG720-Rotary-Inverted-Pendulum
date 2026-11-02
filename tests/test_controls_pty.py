"""Real Engine and serial recording over a synthetic UART, never opens a USB port."""

import json, os, pty, select, struct, sys, tempfile, threading, time, unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Mac上位机"))
import server
from protocol_vendor import crc16


class SerialControl(unittest.TestCase):
    def test_engine_claim_start_task_heartbeat_stop_and_log(self):
        master, slave = pty.openpty()
        path = os.ttyname(slave)
        old = server.ports
        server.ports = lambda: [path]
        ended = threading.Event()
        seen = []
        buf = bytearray()
        errors = []
        e = None
        base = bytearray((ROOT / "tests/remote_uart_vectors.bin").read_bytes()[:168])
        t0 = time.monotonic()
        state = dict(sid=0, seq=0, op=0, owned=0, core=1, mode=0, cfg=0, trial=0)

        def board():
            n = 0
            while not ended.wait(0.02):
                try:
                    if select.select([master], [], [], 0)[0]:
                        buf.extend(os.read(master, 4096))
                    while len(buf) >= 24:
                        if buf[:4] != b"\xaa\xc4\x01\x18":
                            del buf[0]
                            continue
                        b = bytes(buf[:24])
                        del buf[:24]
                        if crc16(b[:20]) != int.from_bytes(b[20:22], "big"):
                            errors.append("CRC")
                            continue
                        op = b[4]
                        arg = int.from_bytes(b[12:16], "big")
                        seen.append(op)
                        state.update(
                            sid=int.from_bytes(b[8:12], "big"),
                            seq=int.from_bytes(b[6:8], "big"),
                            op=op,
                        )
                        if op == 0:
                            state["owned"] = 1
                        elif op == 2:
                            state.update(core=4, trial=state["trial"] + 1)
                        elif op == 3:
                            state.update(mode=1, cfg=arg)
                        elif op == 7:
                            state.update(owned=0, core=1)
                    b = bytearray(base)
                    b[8:12] = n.to_bytes(4, "big")
                    b[12:16] = round((time.monotonic() - t0) * 1000).to_bytes(4, "big")
                    b[16:20] = state["trial"].to_bytes(4, "big")
                    b[21] = state["core"]
                    b[137] = 9 if state["core"] == 4 else 0
                    b[143] = 0
                    b[152:168] = struct.pack(
                        ">IHBBBBBH3s",
                        state["sid"],
                        state["seq"],
                        state["op"],
                        0,
                        12 | state["owned"] | (2 if state["core"] == 4 else 0),
                        state["mode"],
                        state["cfg"],
                        10,
                        b"\0" * 3,
                    )
                    b += struct.pack(">H", crc16(b)) + b"\r\n"
                    os.write(master, b)
                    n += 1
                except OSError:
                    pass

        worker = threading.Thread(target=board, daemon=True)
        worker.start()

        def wait(fn, timeout=3):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                with e.lock:
                    if fn():
                        return
                    if e.control.phase in ("active", "claiming"):
                        e.control.pulse("browser01")
                time.sleep(0.02)
            self.fail("timeout " + str(e.control.status()))

        try:
            with tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                (root / "data").mkdir()
                (root / "data/catalog.json").write_text("[]")
                e = server.Engine(root)
                e.connect(path)
                wait(lambda: e.latest is not None)
                time.sleep(0.15)
                self.assertEqual(seen, [], "connecting must not transmit commands")
                with e.lock:
                    e.control.claim(e, "browser01")
                wait(lambda: e.control.phase == "active")
                with e.lock:
                    e.control.action(e, "browser01", "start", 0)
                wait(
                    lambda: e.latest["state"] == 4
                    and not e.control.waiting
                    and e.control.pending is None
                )
                with e.lock:
                    e.control.action(e, "browser01", "position", 5)
                wait(
                    lambda: e.latest["remote_cfg"] == 5
                    and not e.control.waiting
                    and e.control.pending is None
                )
                for _ in range(5):
                    with e.lock:
                        e.control.pulse("browser01")
                    time.sleep(0.1)
                with e.lock:
                    e.control.stop(e)
                wait(lambda: e.control.phase == "idle")
                time.sleep(0.1)
                self.assertEqual(seen.count(2), 1)
                self.assertEqual(seen.count(3), 1)
                self.assertEqual(seen.count(7), 1)
                self.assertGreaterEqual(seen.count(1), 3)
                self.assertEqual(errors, [])
                self.assertEqual(e.quality.get("bad_crc"), 0)
                e.stop()
                log = [
                    json.loads(x)
                    for x in next((root / "recordings").glob("*.commands.jsonl"))
                    .read_text()
                    .splitlines()
                ]
                self.assertTrue(
                    any(x.get("scope") == "control" and x.get("op") == 2 for x in log)
                )
                self.assertGreater(
                    next((root / "recordings").glob("*.bin")).stat().st_size, 172
                )
        finally:
            if e:
                e.stop()
                e.history.pool.shutdown(wait=True)
            ended.set()
            worker.join(1)
            os.close(master)
            os.close(slave)
            server.ports = old


if __name__ == "__main__":
    unittest.main()
