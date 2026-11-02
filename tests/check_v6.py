from pathlib import Path
import sys, json

r = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(r / "Mac上位机"))
from telemetry import Decoder, normalize

b = bytes.fromhex((r / "tests/flip_uart_vectors.hex").read_text())
d = Decoder()
rows = []
for x in b:
    rows += d.feed(bytes([x]))
assert len(rows) == 6 and d.bad_crc == 0 and d.missing == 0
for a, z in zip(rows, rows[1:]):
    assert z["uptime_ms"] - a["uptime_ms"] == 20
for f in rows:
    assert (
        f["build_id"] == "TR04"
        and f["flip_phase"] == 4
        and f["flip_id"] == 3
        and f["flip_progress"] == 26405
        and f["flip_elapsed_ms"] == 1578
    )
    n = normalize(f)
    assert n["known"] and n["counts_per_rev"] == 1040 and n["protocol_version"] == 6
    assert n["route"] == "WT01 · PD"
(r / "tests/flip_uart_vectors.bin").write_bytes(b)
(r / "reports/v6_protocol.json").write_text(
    json.dumps(
        dict(passed=True, frames=6, size=141, report_hz=50, quality=d.quality()),
        indent=2,
    )
    + "\n"
)
print("PASS real RTL V6 -> Python CRC/fragmented/signed/progress/50Hz decode")
