"""Build frozen sources and audit actual50MHz timing, without opening hardware."""

import argparse, datetime, hashlib, json, os, subprocess, sys, time
from pathlib import Path
from analyze_vendor_timing import analyze

ROOT = Path(__file__).resolve().parent.parent
PROJECT = ROOT / "fpga"


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def inputs():
    paths = list((PROJECT / "src").glob("*.v")) + list(
        (PROJECT / "constraints").glob("*")
    )
    paths += [PROJECT / "acg720_one_euro_WT01E2.gprj", PROJECT / "build.tcl"]
    return {str(p.relative_to(ROOT)): sha(p) for p in sorted(paths)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--ide",
        type=Path,
        default=Path("/Applications/GowinIDE.app/Contents/Resources/Gowin_EDA/IDE"),
    )
    ap.add_argument("--prefix", default="vendor_verified")
    args = ap.parse_args()
    out = ROOT / "reports"
    out.mkdir(exist_ok=True)
    log = out / (args.prefix + ".log")
    meta = out / (args.prefix + ".json")
    if log.exists() or meta.exists():
        raise FileExistsError("Use a new --prefix to preserve previous evidence")
    exe = args.ide / "bin" / ("gw_sh.exe" if os.name == "nt" else "gw_sh")
    env = dict(os.environ)
    if sys.platform == "darwin":
        env.update(
            DYLD_FRAMEWORK_PATH=str(args.ide / "lib"),
            DYLD_LIBRARY_PATH=str(args.ide / "lib"),
        )
    before = inputs()
    started = time.time()
    result = dict(
        started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        inputs_before=before,
        tool=str(exe),
        hardware_access=False,
    )
    print("Gowin synthesis and placement/routing running", flush=True)
    with log.open("w") as f:
        p = subprocess.run(
            [str(exe), str(PROJECT / "build.tcl")],
            cwd=PROJECT,
            env=env,
            stdout=f,
            stderr=subprocess.STDOUT,
        )
    result.update(
        return_code=p.returncode,
        elapsed_seconds=time.time() - started,
        inputs_after=inputs(),
    )
    result["inputs_unchanged"] = before == result["inputs_after"]
    pnr = PROJECT / "impl/pnr"
    stem = "acg720_one_euro_WT01E2"
    fs = pnr / (stem + ".fs")
    result["new_bitstream"] = fs.exists() and fs.stat().st_mtime >= started
    if result["new_bitstream"]:
        result["bitstream_sha256"] = sha(fs)
    if p.returncode == 0 and result["new_bitstream"]:
        result["timing"] = analyze(pnr, stem)
    result["passed"] = bool(
        p.returncode == 0
        and result["inputs_unchanged"]
        and result["new_bitstream"]
        and result.get("timing", {}).get("timing_passed")
        and "Bitstream generation completed" in log.read_text()
    )
    meta.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2), flush=True)
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
