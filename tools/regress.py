#!/usr/bin/env python3
"""Checks that the shipped engines still behave as they did: dyno torque, idle, and sound level
and brightness, measured by engine-sim-bench (repeatable, faster than real time).

  tools/regress.py                 # compare with tests/engine-baseline.json
  tools/regress.py --update        # accept the current results as the new baseline
  tools/regress.py --wav renders/  # also render each engine's run-up to listen to

Runs in temporary config/data dirs, so it never touches your settings or a running engine-sim.
"""

import argparse
import concurrent.futures
import json
import os
import pathlib
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parent.parent
BASELINE = ROOT / "tests" / "engine-baseline.json"
SKIP = {"radial.mr"}  # included by the radial engines, not an engine itself


def engines():
    # Engine files with their own main(); the rest are parts for other scripts to use
    found = (ROOT / "data" / "assets" / "engines").rglob("*.mr")
    return sorted(str(p.relative_to(ROOT)) for p in found if p.name not in SKIP and "node main" in p.read_text(errors="replace"))


def bench(binary, script, wav_dir):
    with tempfile.TemporaryDirectory() as tmp:
        env = dict(os.environ)
        for name in ("XDG_CONFIG_HOME", "XDG_DATA_HOME", "XDG_STATE_HOME", "XDG_RUNTIME_DIR"):
            path = os.path.join(tmp, name.lower())
            os.makedirs(path, mode=0o700)
            env[name] = path
        out = os.path.join(tmp, "result.json")
        command = [binary, "--json", out, script]
        if wav_dir:
            command[1:1] = ["--wav", wav_dir]
        run = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True)
        try:
            with open(out) as f:
                return json.load(f)[0]
        except (OSError, ValueError, IndexError):
            return {"script": script, "error": "bench failed:\n" + run.stdout + run.stderr}


def compare(now, before, tolerance):
    ok = True
    old = {e["script"]: e for e in before}
    for e in now:
        b = old.get(e["script"])
        name = e["script"]
        if b is None:
            print(f"  {name}: new")
            continue
        if ("error" in e) != ("error" in b):
            print(f"  {name}: {'now FAILS to load' if 'error' in e else 'now loads'}")
            ok = False
            continue
        if "error" in e:
            continue

        peak = max(1.0, max(abs(p["torque"]) for p in b["dyno"]))
        worst, worst_rpm = 0.0, 0
        for p, q in zip(e["dyno"], b["dyno"]):
            change = abs(p["torque"] - q["torque"]) / peak * 100
            if change > worst:
                worst, worst_rpm = change, p["rpm"]
        loud = max((abs(p["loudness"] / q["loudness"] - 1) * 100 if q["loudness"] else 0) for p, q in zip(e["dyno"], b["dyno"]))
        idle_now, idle_before = e["idle"]["rpm"], b["idle"]["rpm"]
        idle_moved = abs(idle_now - idle_before) > max(50, idle_before * 0.05)
        moved = worst > tolerance or idle_moved or len(e["dyno"]) != len(b["dyno"])
        ok = ok and not moved
        print(f"  {'CHANGED' if moved else 'same   '} {name:52} torque {worst:5.1f}% (at {worst_rpm:5.0f} rpm)  idle {idle_before:5.0f} -> {idle_now:5.0f}  loudness {loud:5.1f}%")
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--build", default=str(ROOT / "build"), help="meson build dir")
    parser.add_argument("--update", action="store_true", help="save the results as the new baseline")
    parser.add_argument("--tolerance", type=float, default=2.0, help="allowed torque change, percent of peak")
    parser.add_argument("--wav", help="also render each engine to this directory")
    parser.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    parser.add_argument("engines", nargs="*", help="engine files (default: all shipped engines)")
    args = parser.parse_args()

    binary = str(pathlib.Path(args.build) / "frontend" / "bench" / "engine-sim-bench")
    if not os.path.exists(binary):
        sys.exit(f"{binary} not found; build first (ninja -C build)")
    wav_dir = os.path.abspath(args.wav) if args.wav else None

    scripts = args.engines or engines()
    with concurrent.futures.ThreadPoolExecutor(args.jobs) as pool:
        results = list(pool.map(lambda s: bench(binary, s, wav_dir), scripts))
    for r in results:
        r.pop("speed", None)  # machine-dependent
        r.pop("wav", None)

    failed = [r for r in results if "error" in r]
    for r in failed:
        print(f"{r['script']}: {r['error'].strip().splitlines()[0] if r['error'].strip() else 'error'}")

    if args.update:
        if args.engines:
            # Only replace the engines that were run
            before = json.loads(BASELINE.read_text()) if BASELINE.exists() else []
            merged = {e["script"]: e for e in before}
            merged.update({e["script"]: e for e in results})
            results = [merged[k] for k in sorted(merged)]
        BASELINE.write_text(json.dumps(results, indent=1) + "\n")
        print(f"baseline saved: {BASELINE.relative_to(ROOT)} ({len(results)} engines)")
        return 0

    if not BASELINE.exists():
        sys.exit("no baseline yet; run with --update")
    print(f"compared with {BASELINE.relative_to(ROOT)}:")
    return 0 if compare(results, json.loads(BASELINE.read_text()), args.tolerance) else 1


if __name__ == "__main__":
    sys.exit(main())
