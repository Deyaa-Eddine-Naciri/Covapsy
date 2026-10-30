#!/usr/bin/env python3
"""Automated headless Webots parameter tuner for the CoVAPSy TT-02 controller.

Runs Latin Hypercube Sampling (LHS) trials around the working baseline:
  - max_speed_mps : [2.0, 2.6]
  - steer_gain    : [1.1, 1.35]
  - aim_deep_bias : [0.65, 0.85]
  - ttc_target_s  : [0.85, 1.15]

Trial 1 runs the exact baseline configuration (max_speed_mps=2.0, no random overrides).
Trials 2+ sample around the baseline within the search space.

Usage:
    python3 tune_headless.py
"""

import argparse
import csv
import json
import os
import random
import re
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "common"))
import config as _cfg  # noqa: E402

# Working baseline config (loaded directly from common/config.py defaults)
BASELINE_PARAMS = {
    "max_speed_mps": float(_cfg.PARAMS["max_speed_mps"]),
    "min_speed_mps": float(_cfg.PARAMS["min_speed_mps"]),
    "steer_gain": float(_cfg.PARAMS["steer_gain"]),
    "aim_deep_bias": float(_cfg.PARAMS["aim_deep_bias"]),
    "ttc_target_s": float(_cfg.PARAMS["ttc_target_s"]),
    "max_steer_rate_dps": float(_cfg.PARAMS["max_steer_rate_dps"]),
    "clearance_margin_m": float(_cfg.PARAMS["clearance_margin_m"]),
}

# Expanded search space for optimal tuning exploration
SEARCH_SPACE = {
    "max_speed_mps": (2.2, 3.2),          # cruise pace; explore higher speeds
    "min_speed_mps": (0.6, 1.4),          # cornering minimum speed floor
    "steer_gain": (1.0, 1.7),             # heading responsiveness
    "aim_deep_bias": (0.50, 0.95),        # apex pulling bias
    "ttc_target_s": (0.70, 1.25),         # braking distance margin
    "max_steer_rate_dps": (140.0, 360.0), # steering slew rate limit
    "clearance_margin_m": (0.02, 0.12),   # obstacle / wall inflation berth
}

DEFAULT_NUM_TRIALS = 30
CSV_OUTPUT = Path(__file__).resolve().parent / "tuning_results.csv"
ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-9;]*m")
LAP_TIME_RE = re.compile(r"\[LAP TIMER\]\s+Lap\s+\d+\s+completed:\s+([0-9]+(?:\.[0-9]+)?)\s*s")


def find_world_file(root_dir: Path) -> Path:
    """Locate the .wbt world file inside simulation/worlds/."""
    worlds_dir = root_dir / "simulation" / "worlds"
    if not worlds_dir.is_dir():
        raise FileNotFoundError(f"Could not find worlds directory: {worlds_dir}")

    # Prefer standalone.wbt for single-car tuning; otherwise pick the first .wbt
    standalone = worlds_dir / "standalone.wbt"
    if standalone.is_file():
        return standalone

    wbt_files = sorted(worlds_dir.glob("*.wbt"))
    if not wbt_files:
        raise FileNotFoundError(f"No .wbt files found in {worlds_dir}")
    return wbt_files[0]


def latin_hypercube_samples(space: dict, n_samples: int, seed: int = 42) -> list:
    """Generate n_samples parameter dicts using Latin Hypercube Sampling (LHS)."""
    rng = random.Random(seed)
    keys = list(space.keys())
    dim_samples = {}

    for k in keys:
        lo, hi = space[k]
        # Stratify [0, 1] into n_samples equal intervals and sample uniformly within each
        strata = [(i + rng.random()) / n_samples for i in range(n_samples)]
        rng.shuffle(strata)
        dim_samples[k] = [round(lo + s * (hi - lo), 4) for s in strata]

    trials = []
    for i in range(n_samples):
        trials.append({k: dim_samples[k][i] for k in keys})
    return trials


DNF_REASON_RE = re.compile(r"\[TUNE_RESULT\]\s+DNF(?:\s*\(([^)]+)\))?")


def parse_simulation_output(raw_output: str):
    """Extract (lap_time, dnf_reason) from controller stdout/logfile."""
    clean = ANSI_ESCAPE_RE.sub("", raw_output)
    dnf_match = DNF_REASON_RE.search(clean)
    if dnf_match:
        return None, (dnf_match.group(1) or "aborted")

    match = LAP_TIME_RE.search(clean)
    if match:
        return float(match.group(1)), None
    return None, "no lap time found in output"


def save_csv(results: list, csv_path: Path):
    """Write all completed trials to tuning_results.csv."""
    if not results:
        return
    param_keys = [k for k in results[0].keys() if k not in ("trial", "status", "lap_time_s")]
    fieldnames = ["trial", "status", "lap_time_s"] + param_keys
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            row = {
                "trial": r["trial"],
                "status": r["status"],
                "lap_time_s": f"{r['lap_time_s']:.3f}" if r["lap_time_s"] is not None else "DNF",
            }
            for k in param_keys:
                v = r.get(k)
                row[k] = f"{v:.4f}" if isinstance(v, float) else str(v)
            writer.writerow(row)


def print_leaderboard(results: list, total_trials: int, top_k: int = 10):
    """Print a live leaderboard of the fastest completed runs so far."""
    completed = [r for r in results if r["lap_time_s"] is not None]
    completed.sort(key=lambda r: r["lap_time_s"])
    dnf_count = len(results) - len(completed)

    print("\n" + "=" * 90)
    print(f"  LIVE LEADERBOARD ({len(results)}/{total_trials} runs | "
          f"{len(completed)} finished | {dnf_count} DNF)")
    print("-" * 90)
    if not completed:
        print("  No valid laps completed yet.")
    else:
        print(f"  {'Rank':<5} {'Trial':<6} {'Lap Time':<11} "
              f"{'max_spd':<9} {'min_spd':<9} {'steer_g':<9} {'aim_deep':<9} {'ttc_tgt':<9} {'steer_rt':<9}")
        for rank, r in enumerate(completed[:top_k], start=1):
            marker = " *" if rank == 1 else ""
            print(f"  #{rank:<4} #{r['trial']:<5} {r['lap_time_s']:>6.2f} s   "
                  f"{r.get('max_speed_mps', 0.0):>7.2f}  "
                  f"{r.get('min_speed_mps', 0.0):>7.2f}  "
                  f"{r.get('steer_gain', 0.0):>7.2f}  "
                  f"{r.get('aim_deep_bias', 0.0):>7.2f}  "
                  f"{r.get('ttc_target_s', 0.0):>7.2f}  "
                  f"{r.get('max_steer_rate_dps', 0.0):>7.1f}{marker}")
    print("=" * 90 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Headless Webots parameter tuner")
    parser.add_argument("--trials", type=int, default=DEFAULT_NUM_TRIALS,
                        help="Number of trials to run (default: 30)")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed for Latin Hypercube Sampling")
    parser.add_argument("--world", type=str, default=None,
                        help="Optional path to .wbt file (defaults to simulation/worlds/standalone.wbt)")
    parser.add_argument("--timeout", type=float, default=65.0,
                        help="Wall-clock timeout per Webots process in seconds (default: 65.0)")
    args = parser.parse_args()

    root_dir = Path(__file__).resolve().parent
    world_path = Path(args.world).resolve() if args.world else find_world_file(root_dir)
    print(f"[TUNE] World file : {world_path}")
    print(f"[TUNE] CSV output : {CSV_OUTPUT}")
    print(f"[TUNE] Trials     : {args.trials} (Trial 1 = baseline, Trials 2+ = LHS, seed={args.seed})\n")

    if args.trials > 1:
        sampled = latin_hypercube_samples(SEARCH_SPACE, args.trials - 1, seed=args.seed)
        trials = [dict(BASELINE_PARAMS)] + sampled
    else:
        trials = [dict(BASELINE_PARAMS)]

    results = []

    for idx, params in enumerate(trials, start=1):
        is_baseline = (idx == 1)
        tag = "(BASELINE)" if is_baseline else "Testing:"
        param_str = ", ".join(f"{k}={v:.3f}" if isinstance(v, float) else f"{k}={v}" for k, v in params.items())
        print(f"[Trial {idx:02d}/{args.trials:02d}] {tag} {param_str} ...")

        # Place log and override files inside project dir (Snap Webots uses
        # a private /tmp namespace that hides /tmp files from the host).
        log_path = str(root_dir / ".covapsy_tune.log")
        overrides_path = root_dir / "common" / ".tune_overrides.json"
        if os.path.exists(log_path):
            os.remove(log_path)

        env = os.environ.copy()
        env["COVAPSY_AUTOSTART"] = "1"
        env["COVAPSY_HEADLESS_TUNE"] = "1"
        env["COVAPSY_LOGFILE"] = log_path
        # Force NVIDIA dGPU render offload on hybrid/Optimus systems
        env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
        env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"

        tune_dict = {}
        if not is_baseline:
            for k, v in params.items():
                if k == "max_steer_deg":
                    v = min(float(v), 18.0)
                env[f"TUNE_{k}"] = str(v)
                tune_dict[k] = v

        with open(overrides_path, "w") as ovf:
            json.dump({
                "env": {
                    "COVAPSY_AUTOSTART": "1",
                    "COVAPSY_HEADLESS_TUNE": "1",
                    "COVAPSY_LOGFILE": log_path,
                    "__NV_PRIME_RENDER_OFFLOAD": "1",
                    "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
                },
                "tune": tune_dict,
            }, ovf)

        cmd = [
            "webots",
            "--batch",
            "--mode=fast",
            "--stdout",
            str(world_path),
        ]

        lap_time = None
        dnf_reason = None
        combined_out = ""
        timed_out = False
        try:
            proc = subprocess.run(
                cmd,
                env=env,
                capture_output=True,
                text=True,
                timeout=args.timeout,
            )
            combined_out = (proc.stdout or "") + "\n" + (proc.stderr or "")
        except subprocess.TimeoutExpired as exc:
            timed_out = True
            out_s = exc.stdout.decode("utf-8", errors="replace") if isinstance(exc.stdout, bytes) else (exc.stdout or "")
            err_s = exc.stderr.decode("utf-8", errors="replace") if isinstance(exc.stderr, bytes) else (exc.stderr or "")
            combined_out = out_s + "\n" + err_s
        except FileNotFoundError:
            print("  -> [ERROR] 'webots' binary not found in PATH.")
            sys.exit(1)
        finally:
            if os.path.exists(log_path):
                with open(log_path, "r") as lf:
                    combined_out += "\n" + lf.read()
                os.remove(log_path)
            if overrides_path.exists():
                overrides_path.unlink()

        lap_time, dnf_reason = parse_simulation_output(combined_out)
        if timed_out and lap_time is None and dnf_reason == "no lap time found in output":
            dnf_reason = f"wall-clock timeout > {args.timeout:.0f}s"
            print(f"  -> [TIMEOUT] Webots process exceeded {args.timeout:.0f}s wall-clock limit.")

        status = "OK" if lap_time is not None else "DNF"
        if lap_time is not None:
            print(f"  -> Completed Lap 1 in {lap_time:.2f} s")
        else:
            reason_str = f" ({dnf_reason})" if dnf_reason else ""
            print(f"  -> Result: [TUNE_RESULT] DNF{reason_str}")

        record = {
            "trial": idx,
            "status": status,
            "lap_time_s": lap_time,
            **params,
        }
        results.append(record)
        save_csv(results, CSV_OUTPUT)
        print_leaderboard(results, args.trials)

    print(f"[TUNE] Finished all {args.trials} trials. Results saved to {CSV_OUTPUT}")


if __name__ == "__main__":
    main()

