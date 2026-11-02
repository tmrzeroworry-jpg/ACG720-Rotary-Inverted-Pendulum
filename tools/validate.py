#!/usr/bin/env python3
"""Run the UI02 offline regression in a temporary copy, without accessing USB.

Requirements: Python 3.9+, Icarus Verilog (iverilog/vvp) and Node.js 18+ on PATH.
Generated UART fixtures, traces and PTY captures never modify the working tree.
"""

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
BENCHES = (
    "tb_curve_handoff",
    "tb_curve_limits",
    "tb_remote_fourpins",
    "tb_remote_control",
    "tb_remote_hold",
    "tb_remote_uart",
    "tb_transition_seam",
    "tb_transition_core",
    "tb_transition_unit",
    "tb_transition_routes",
    "tb_return_link",
    "tb_flip_link",
    "tb_flip_variants",
    "tb_stream_reference",
    "tb_velocity_stream",
    "tb_uart_rx",
    "tb_flip_sensor",
    "tb_life_uart",
    "tb_transition_plant",
    "tb_legacy_equivalence",
)


def main():
    executables = {name: shutil.which(name) for name in ("iverilog", "vvp", "node")}
    missing = [name for name, path in executables.items() if path is None]
    if missing:
        raise SystemExit("Missing tools on PATH: " + ", ".join(missing))
    results = []
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    # Synthetic loopback tests must not be routed through a system HTTP proxy.
    env.update(NO_PROXY="127.0.0.1,localhost", no_proxy="127.0.0.1,localhost")

    with tempfile.TemporaryDirectory(prefix="ui02_offline_") as tmp:
        stage = Path(tmp)
        for name in ("fpga", "tests", "Mac上位机"):
            shutil.copytree(
                ROOT / name,
                stage / name,
                ignore=shutil.ignore_patterns(
                    "__pycache__",
                    "*.pyc",
                    "recordings",
                    "logs",
                    "impl",
                    "*.gprj.user",
                ),
            )
        (stage / "reports").mkdir()
        sources = sorted((stage / "fpga/src").glob("*.v"))

        def run(name, args):
            try:
                proc = subprocess.run(
                    [str(x) for x in args],
                    cwd=stage,
                    env=env,
                    capture_output=True,
                    text=True,
                    timeout=150,
                )
                result = dict(
                    name=name,
                    return_code=proc.returncode,
                    output=proc.stdout + proc.stderr,
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                result = dict(name=name, return_code=1, output=str(exc))
            results.append(result)
            print(
                name + (" PASS" if result["return_code"] == 0 else " FAIL"), flush=True
            )
            if result["return_code"]:
                print(result["output"], flush=True)
            return result["return_code"] == 0

        project = ET.parse(stage / "fpga/acg720_one_euro_WT01E2.gprj")
        listed = {entry.attrib["path"] for entry in project.findall("./FileList/File")}
        actual = {str(p.relative_to(stage / "fpga")) for p in sources}
        assert {p for p in listed if p.endswith(".v")} == actual
        assert all((stage / "fpga" / p).is_file() for p in listed)
        results.append(dict(name="project source list and constraints", return_code=0))
        iv, vvp, node = (executables[name] for name in ("iverilog", "vvp", "node"))
        run(
            "top-level compile",
            [iv, "-g2012", "-s", "ab4_top", "-o", stage / "top", *sources],
        )
        for name in BENCHES:
            target = stage / name
            gold = (
                [stage / "tests/swing_controller_gold.v"]
                if name == "tb_legacy_equivalence"
                else []
            )
            if not run(
                name + " compile",
                [
                    iv,
                    "-g2012",
                    "-s",
                    name,
                    "-o",
                    target,
                    stage / "tests" / (name + ".v"),
                    *sources,
                    *gold,
                ],
            ):
                continue
            variants = [[]]
            if name == "tb_transition_seam":
                variants = [["+CASE=" + str(case)] for case in range(6)]
            elif name == "tb_transition_core":
                variants = [["+FAILSEAM=" + str(case)] for case in range(2)]
            elif name == "tb_transition_plant":
                variants = [
                    [f"+MODEL={m}", f"+ACTION={a}", f"+SIDE={s}"]
                    for m, a, s in (
                        (0, 0, 1),
                        (1, 0, 1),
                        (0, 1, 1),
                        (0, 1, -1),
                        (1, 1, 1),
                        (1, 1, -1),
                    )
                ]
            for variant in variants:
                run(" ".join([name, *variant]), [vvp, target, *variant])

        # Preserve the existing real-serializer assertions, not handcrafted packets.
        sys.path.insert(0, str(stage / "Mac上位机"))
        from protocol_vendor import decode_frame

        try:
            remote = bytes.fromhex(
                (stage / "tests/remote_uart_vectors.hex").read_text()
            )
            assert len(remote) == 1032
            (stage / "tests/remote_uart_vectors.bin").write_bytes(remote)
            data = bytes.fromhex((stage / "tests/life_uart_vectors.hex").read_text())
            assert len(data) == 936
            for n in range(0, len(data), 156):
                f = decode_frame(data[n : n + 156])
                assert tuple(
                    f[key]
                    for key in (
                        "build_id",
                        "protocol_version",
                        "life_phase",
                        "life_id",
                        "life_elapsed_ms",
                        "life_mode",
                        "life_turns",
                        "life_down",
                        "life_down_valid",
                        "life_rate",
                    )
                ) == ("TR06", 7, 7, 9, 2384, 1, 2, 20000, True, -1200)
            (stage / "tests/life_uart_vectors.bin").write_bytes(data)
            results.append(dict(name="V7/V8 actual serializer fixtures", return_code=0))
        except (OSError, ValueError, AssertionError) as exc:
            results.append(
                dict(
                    name="V7/V8 actual serializer fixtures",
                    return_code=1,
                    output=repr(exc),
                )
            )

        for directory in ("tests", "Mac上位机/tests"):
            run(
                "Python suite: " + directory,
                [
                    sys.executable,
                    "-m",
                    "unittest",
                    "discover",
                    "-s",
                    directory,
                    "-p",
                    "test_*.py",
                    "-v",
                ],
            )
        for file in sorted((stage / "Mac上位机/web").glob("*.js")):
            run("JS syntax: " + file.name, [node, "--check", file])
        for file in sorted((stage / "Mac上位机/tests").glob("test_*.js")):
            run("JS assertions: " + file.name, [node, file])

    passed = all(item["return_code"] == 0 for item in results)
    output = ROOT / "reports/offline_validation.json"
    output.parent.mkdir(exist_ok=True)
    summary = dict(
        passed=passed,
        hardware_access=False,
        source_sha256={
            str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted((ROOT / "fpga/src").glob("*.v"))
        },
        tests=results,
    )
    output.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print("ALL PASS" if passed else "FAILURES: see reports/offline_validation.json")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
