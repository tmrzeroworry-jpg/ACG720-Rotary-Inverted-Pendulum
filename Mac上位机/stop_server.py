"""Stop only the PC01 instance belonging to this exact folder."""

import json, os, signal, urllib.request
from pathlib import Path

root = Path(__file__).resolve().parent
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
found = False
for port in range(8771, 8781):
    base = "http://127.0.0.1:" + str(port)
    try:
        with opener.open(base + "/api/health", timeout=0.3) as r:
            health = json.load(r)
        if health.get("app") != "RIP_UI02_LIVE_MAC" or health.get("root") != str(root):
            continue
        request = urllib.request.Request(
            base + "/api/disconnect",
            data=b"{}",
            headers={"Content-Type": "application/json"},
        )
        with opener.open(request, timeout=4) as r:
            json.load(r)
        os.kill(health["PID"], signal.SIGTERM)
        found = True
    except (OSError, ValueError):
        continue
print("上位机已停止，接收记录已保存。" if found else "当前目录的上位机没有运行。")
