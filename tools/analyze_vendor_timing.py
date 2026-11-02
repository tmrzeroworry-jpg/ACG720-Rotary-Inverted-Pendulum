"""Audit GoWin's actual report content, independently of CLI return code."""

import argparse, json, re
from html.parser import HTMLParser
from pathlib import Path


class ReportText(HTMLParser):
    def __init__(self):
        super().__init__()
        self.parts = []

    def handle_data(self, value):
        if value.strip():
            self.parts.append(value.strip())


def analyze(folder, stem):
    parser = ReportText()
    parser.feed((folder / f"{stem}_tr_content.html").read_text())

    def count(label):
        return int(parser.parts[parser.parts.index(label) + 1])

    paths = []
    for block in re.split(r"={5,}", (folder / f"{stem}.timing_paths").read_text()):
        lines = block.strip().splitlines()
        if not lines or lines[0] not in ("SETUP", "HOLD"):
            continue
        names = []
        for item in lines[4:]:
            try:
                float(item)
            except ValueError:
                names.append(item)
        paths.append(
            {
                "type": lines[0],
                "slack_ns": float(lines[1]),
                "source": names[1] if len(names) > 1 else None,
                "destination": names[-1],
            }
        )
    setup = min(p["slack_ns"] for p in paths if p["type"] == "SETUP")
    hold = min(p["slack_ns"] for p in paths if p["type"] == "HOLD")
    sv = count("Numbers of Setup Violated Endpoints")
    hv = count("Numbers of Hold Violated Endpoints")
    period = 20.0
    for sdc in (folder.parents[1] / "constraints").glob("*.sdc"):
        m = re.search(r"create_clock[^\n]*-period\s+([0-9.]+)", sdc.read_text())
        if m:
            period = float(m.group(1))
    return {
        "clock_frequency_MHz": 50,
        "actual_board_clock_MHz": 50,
        "constraint_period_ns": period,
        "constraint_frequency_MHz": 1000 / period,
        "worst_setup_slack_at_board_20ns_ns": round(setup + 20 - period, 3),
        "slack_note": "worst_setup_slack_ns refers to PNR constraint; actual oscillator remains20ns",
        "setup_violations": sv,
        "hold_violations": hv,
        "worst_setup_slack_ns": setup,
        "worst_hold_slack_ns": hold,
        "timing_passed": sv == 0 and hv == 0 and setup >= 0 and hold >= 0,
        "path_examples": paths[:3],
    }


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--folder", type=Path, required=True)
    ap.add_argument("--stem", required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    result = analyze(args.folder, args.stem)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))
    raise SystemExit(0 if result["timing_passed"] else 1)
