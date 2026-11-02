"""Local commissioning helper. Requires an already-authorized SRAM download.

The vendor CLI may return zero on a verification error; inspect its full log.
Never performs Flash or OTP writes. Physical supply state is confirmed by chat.
"""

import argparse
import hashlib
import json
import re
import subprocess
import time
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--fs", required=True)
ap.add_argument("--prefix", required=True)
ap.add_argument(
    "--location",
    type=int,
    default=259,
    help="Use the location from the current programmer --scan-cables F result.",
)
args = ap.parse_args()
fs = Path(args.fs).resolve()
prefix = Path(args.prefix)
prefix.parent.mkdir(parents=True, exist_ok=True)
cmd = [
    "/Applications/GowinIDE.app/Contents/Resources/Gowin_EDA/Programmer/bin/programmer_cli",
    "--run",
    "4",
    "--cable-index",
    "1",
    "--location",
    str(args.location),
    "--device",
    "GW5AT-138C",
    "--frequency",
    "2.5MHz",
    "--fsFile",
    str(fs),
]
start = time.monotonic()
proc = subprocess.Popen(
    cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1
)
lines = []
seen = set()
condensed = []
with prefix.with_suffix(".log").open("w") as log:
    for line in proc.stdout:
        lines.append(line)
        log.write(line)
        m = re.search(r"(Programing|Programming|Reading SRAM)\.*:\s*.*?(\d+)%", line)
        if m:
            key = (m.group(1), int(m.group(2)))
            if key[1] % 25 == 0 and key not in seen:
                message = key[0] + " " + str(key[1]) + "%"
                seen.add(key)
            else:
                continue
        else:
            message = line.strip()
            if not message:
                continue
        print(message, flush=True)
        condensed.append(message)
rc = proc.wait()
s = "".join(lines)
status = re.search(r"Status Code is:\s*(0x[0-9A-Fa-f]+)", s)
user = re.search(r"User Code is:\s*(0x[0-9A-Fa-f]+)", s)
expected = re.search(r"//UserCode:\s*(0x[0-9A-Fa-f]+)", fs.read_text()[:4096])
errors = [line.strip() for line in lines if re.search(r"error|fail", line, re.I)]
sv = int(status.group(1), 16) if status else None
matched = bool(
    user and expected and int(user.group(1), 16) == int(expected.group(1), 16)
)
verified = bool(
    rc == 0
    and not errors
    and re.search(r"Reading SRAM.*100%", s)
    and "Finished." in s
    and sv is not None
    and (sv & (1 << 13))
    and not (sv & (1 << 14))
    and not (sv & 15)
    and matched
)
report = {
    "method": "SRAM Program and Verify, operation4",
    "return_code": rc,
    "cable_location": args.location,
    "verify_passed": verified,
    "errors": errors,
    "elapsed_s": round(time.monotonic() - start, 2),
    "bitstream_sha256": hashlib.sha256(fs.read_bytes()).hexdigest(),
    "bitstream_path": str(fs),
    "status_code": status.group(1) if status else None,
    "user_code": user.group(1) if user else None,
    "user_code_matches_bitstream": matched,
    "security_bit": bool(sv & (1 << 14)) if sv is not None else None,
    "program_log": str(prefix.with_suffix(".log")),
    "verify_evidence": (
        "Readback100%; no error; Finished; Done bit13=1, security bit14=0, error bits0..3=0; UserCode matches FS header."
        if verified
        else "Verification not established; inspect errors, status and full programming log."
    ),
}
prefix.with_suffix(".json").write_text(
    json.dumps(report, ensure_ascii=False, indent=2) + "\n"
)
prefix.with_suffix(".summary.txt").write_text("\n".join(condensed) + "\n")
print("VERIFY_AUDIT " + json.dumps(report, ensure_ascii=False), flush=True)
raise SystemExit(0 if verified else 1)
