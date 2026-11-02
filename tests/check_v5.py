import sys, json
from pathlib import Path

r = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(r / "Mac上位机"))
from telemetry import Decoder, normalize
from history_export import HistoryExport

b = bytes.fromhex((r / "tests/stream_uart_vectors.hex").read_text())
d = Decoder()
f = []
for x in b:
    f += d.feed(bytes([x]))
assert len(f) == 6 and d.bad_crc == 0 and d.missing == 0, d.quality()
for a, z in zip(f, f[1:]):
    assert z["uptime_ms"] - a["uptime_ms"] == 20
for v in f:
    assert (
        v["build_id"] == "TR01"
        and v["stream_session"] == 0x12345678
        and v["stream_seq"] == 65535
    )
    assert (
        v["stream_velocity_q16"] == -8519
        and v["stream_desired_q16"] == 17039
        and v["stream_age_ms"] == 42
    )
    n = normalize(v)
    assert n["known"] and n["counts_per_rev"] == 1040 and n["protocol_version"] == 5
(r / "tests/stream_uart_vectors.bin").write_bytes(b)
(r / "reports/v5_protocol.json").write_text(
    json.dumps(
        dict(passed=True, frames=6, frame_bytes=131, report_hz=50, quality=d.quality()),
        indent=2,
    )
    + "\n"
)
print(
    "PASS RTL V5 -> Python fragmented decode, signed fields, CRC, 50Hz, correct1040scale"
)
