#!/usr/bin/env python3
"""Launch this local app or reopen an existing instance of the same folder."""

import json
import socket
import sys
import urllib.request
import webbrowser
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def main():
    for port in range(8777, 8787):
        url = "http://127.0.0.1:%d" % port
        try:
            # Ignore system proxy settings for the local health check.
            opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            with opener.open(url + "/api/health", timeout=0.3) as r:
                health = json.load(r)
            if health.get("app") == "RIP_UI02_LIVE_MAC" and health.get("root") == str(
                ROOT
            ):
                webbrowser.open(url)
                print("已打开正在运行的上位机：" + url)
                return
        except (OSError, ValueError):
            pass
    for port in range(8777, 8787):
        url = "http://127.0.0.1:%d" % port
        try:
            with socket.socket() as probe:
                probe.bind(("127.0.0.1", port))
        except OSError:
            continue
        logs = ROOT / "logs"
        logs.mkdir(exist_ok=True)
        with (logs / "server.log").open("ab", buffering=0) as output:
            proc = subprocess.Popen(
                [sys.executable, str(ROOT / "server.py"), "--port", str(port)],
                cwd=str(ROOT),
                stdin=subprocess.DEVNULL,
                stdout=output,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        for _ in range(40):
            if proc.poll() is not None:
                break
            try:
                with opener.open(url + "/api/health", timeout=0.3) as r:
                    health = json.load(r)
                if (
                    health.get("app") == "RIP_UI02_LIVE_MAC"
                    and health.get("PID") == proc.pid
                ):
                    webbrowser.open(url)
                    print("上位机已在后台启动：" + url + "；关闭启动窗口不会停止服务。")
                    return
            except (OSError, ValueError):
                pass
            time.sleep(0.1)
        raise SystemExit("服务未就绪，请查看logs/server.log。")
    raise SystemExit("8777～8785端口都被占用，请关闭之前的上位机窗口后重试。")


if __name__ == "__main__":
    main()
