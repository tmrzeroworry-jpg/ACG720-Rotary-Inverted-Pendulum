"""Synthetic PTY board, no USB hardware access. Exercises actual Engine serial_loop."""

import os, pty, select, struct, sys, threading, time, json, tempfile, unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Mac上位机"))
import server
from trajectory import Q
from protocol_vendor import crc16


class E2E(unittest.TestCase):
    def test_duplex_serial_handshake_update_stop_and_record(self):
        root = Path(__file__).resolve().parents[1]
        master, slave = pty.openpty()
        path = os.ttyname(slave)
        old_ports = server.ports
        server.ports = lambda: [path]
        ended = threading.Event()
        seen = []
        state = {"session": 0, "seq": 0, "flags": 1, "v": 0, "desired": 0}
        buf = bytearray()
        base = bytearray(
            (root / "Mac上位机/tests/home_uart_vectors.bin").read_bytes()[:109]
        )
        base[:8] = b"\xaaV\x08\xacUI02"
        base[21] = 4
        start = time.monotonic()

        def board():
            n = 0
            while not ended.wait(0.01):
                try:
                    if select.select([master], [], [], 0)[0]:
                        buf.extend(os.read(master, 4096))
                    while len(buf) >= 24:
                        if buf[:4] != b"\xaa\xc3\x01\x18":
                            del buf[0]
                            continue
                        b = bytes(buf[:24])
                        del buf[:24]
                        if crc16(b[:20]) != int.from_bytes(b[20:22], "big"):
                            continue
                        op = b[4]
                        state["seq"] = int.from_bytes(b[6:8], "big")
                        state["session"] = int.from_bytes(b[8:12], "big")
                        seen.append(op)
                        if op == 0:
                            state["flags"] = 33
                        elif op in (1, 2):
                            state["flags"] = 35
                            state["desired"] = int.from_bytes(
                                b[12:16], "big", signed=True
                            )
                        elif op == 3:
                            state["desired"] = 0
                    state["v"] = state["desired"]
                    b = bytearray(base)
                    b[8:12] = n.to_bytes(4, "big")
                    b[12:16] = round((time.monotonic() - start) * 1000).to_bytes(
                        4, "big"
                    )
                    b[36:40] = round(state["v"] / 65536 * 1000).to_bytes(
                        4, "big", signed=True
                    )
                    b += struct.pack(
                        ">IHBBiiH",
                        state["session"],
                        state["seq"],
                        state["flags"],
                        0,
                        state["v"],
                        state["desired"],
                        10,
                    )
                    b += bytes(41)
                    b += struct.pack(">H", crc16(b)) + b"\r\n"
                    os.write(master, b)
                    n += 1
                except OSError:
                    pass

        worker = threading.Thread(target=board, daemon=True)
        worker.start()
        e = None

        def wait_for(fn, timeout=2):
            until = time.monotonic() + timeout
            while time.monotonic() < until:
                if e:
                    with e.lock:
                        if fn():
                            return
                time.sleep(0.01)
            raise AssertionError(
                "timeout: " + (str(e.status().get("trajectory")) if e else "")
            )

        try:
            with tempfile.TemporaryDirectory() as tmp:
                app = Path(tmp)
                (app / "data").mkdir()
                (app / "data/catalog.json").write_text("[]")
                e = server.Engine(app)
                e.connect(path)
                wait_for(lambda: e.latest is not None)
                with e.lock:
                    e.trajectory.update(
                        dict(points=[[0, 0], [1, 20], [2, 0]], client="page0001")
                    )
                    e.trajectory.start(e, "page0001")
                wait_for(lambda: e.trajectory.phase == "running")
                end = time.monotonic() + 0.4
                while time.monotonic() < end:
                    with e.lock:
                        e.trajectory.pulse("page0001")
                    time.sleep(0.08)
                with e.lock:
                    e.trajectory.update(
                        dict(points=[[0, 0], [1, 30], [2, 0]], client="page0001")
                    )
                    e.trajectory.stop()
                wait_for(lambda: e.trajectory.phase == "done")
                self.assertIn(0, seen)
                self.assertIn(1, seen)
                self.assertIn(2, seen)
                self.assertIn(3, seen)
                e.stop()
                logs = list((app / "recordings").glob("*.commands.jsonl"))
                self.assertEqual(len(logs), 1)
                events = [json.loads(x) for x in logs[0].read_text().splitlines()]
                self.assertTrue(any(x["event"] == "tx" for x in events))
                sessions = list((app / "recordings").glob("*.session.json"))
                self.assertTrue(json.loads(sessions[0].read_text())["transmits_data"])
                self.assertGreater(
                    list((app / "recordings").glob("*.bin"))[0].stat().st_size, 131
                )
        finally:
            if e:
                e.stop()
                e.history.pool.shutdown(wait=True)
            ended.set()
            worker.join(1)
            os.close(master)
            os.close(slave)
            server.ports = old_ports
        (root / "reports/serial_e2e.json").write_text(
            json.dumps(
                dict(
                    passed=True,
                    hardware_access=False,
                    transport="synthetic PTY",
                    seen_commands=seen,
                ),
                indent=2,
            )
            + "\n"
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
