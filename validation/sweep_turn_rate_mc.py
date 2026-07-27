#!/usr/bin/env python3
"""
sweep_turn_rate_mc.py - turn_rate discrete multi-model feasibility test

Each task spawns an independent MATLAB process (avoids heap corruption in long runs).
Results are accumulated incrementally.

Usage: python validation/sweep_turn_rate_mc.py [seed_start] [seed_end]
       e.g.: python validation/sweep_turn_rate_mc.py 1 50
"""
import subprocess
import os
import sys
import time
import re
import math
import tempfile
from pathlib import Path

# Lazy-load scipy only when needed (not required for basic operation)
scipy = None
try:
    import scipy.io
except ImportError:
    pass

MATLAB = r"C:\Program Files\MATLAB\R2023a\bin\matlab.exe"
ROOT = Path(__file__).resolve().parent.parent
RESULTS_DIR = ROOT / "results"
PROGRESS_FILE = RESULTS_DIR / "sweep_progress.csv"

CANDIDATES_DEG = [0.5, 0.7, 1.0, 1.3, 1.5, 2.0]
SCENARIOS = ['right_rate_0p7', 'left_rate_0p7', 'right_rate_1p3', 'left_rate_1p3']
TRUE_RATES = [0.7, 0.7, 1.3, 1.3]


def load_progress():
    """Load completed tasks from CSV."""
    if not PROGRESS_FILE.exists():
        return {}
    done = {}
    with open(PROGRESS_FILE, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) == 4:
                try:
                    key = (int(parts[0]), int(parts[1]), int(parts[2]))
                    done[key] = float(parts[3])
                except ValueError:
                    pass
    return done


def save_done(key, val):
    """Append a completed task to progress file."""
    with open(PROGRESS_FILE, 'a') as f:
        f.write(f"{key[0]},{key[1]},{key[2]},{val}\n")


def run_single(seed, scenario_name, candidate_rate_deg):
    """Run one trial in a standalone MATLAB process.

    Uses fopen to write RMSE value to a temp file, avoiding all
    MATLAB batch-mode quoting/encoding issues.
    """
    rate_rad = candidate_rate_deg * math.pi / 180.0

    tmp_file = f'_val_s{seed}_sc{candidate_rate_deg}.txt'

    # Build MATLAB command using single quotes only (safe for batch mode)
    matlab_cmd = (
        "addpath(genpath('.')); "
        f"params = simulation_params_oracle(); "
        f"params.random_seed = {seed}; "
        f"params.imm_turn_rate_rad_per_sec = {rate_rad}; "
        f"try; r = run('{scenario_name}', params); "
        f"v = r.errorStats_R1.overall.ukf.rms; "
        f"fid = fopen('{tmp_file}', 'w'); "
        f"fprintf(fid, '%f', v); "
        f"fclose(fid); catch; fid = fopen('{tmp_file}', 'w'); "
        f"fprintf(fid, 'NaN'); fclose(fid); end"
    )

    try:
        cmd = [MATLAB, '-batch', matlab_cmd]
        subprocess.run(cmd, capture_output=True, encoding='gbk', errors='replace', timeout=300)

        if os.path.exists(tmp_file):
            with open(tmp_file, 'r') as f:
                content = f.read().strip()
            os.remove(tmp_file)
            if content and content != 'NaN':
                try:
                    return float(content)
                except ValueError:
                    return None
        return None

    except subprocess.TimeoutExpired:
        # Clean up temp file if it exists
        if os.path.exists(tmp_file):
            os.remove(tmp_file)
        return 'timeout'
    except Exception:
        if os.path.exists(tmp_file):
            os.remove(tmp_file)
        return 'error'


def build_rmse_table(done_map, seeds):
    """Build the full rmse_mat array from done_map."""
    n_sc = len(SCENARIOS)
    n_cand = len(CANDIDATES_DEG)
    n_seeds = len(seeds)

    rmse_mat = [[[[None for _ in range(n_cand)] for _ in range(n_seeds)] for _ in range(n_sc)]]

    for (seed, sci, ci), val in done_map.items():
        si = seeds.index(seed) if seed in seeds else -1
        if si >= 0:
            rmse_mat[sci][si][ci-1] = val

    return rmse_mat


def print_summary(done_map, seeds):
    """Print final summary table."""
    import math

    n_sc = len(SCENARIOS)
    n_cand = len(CANDIDATES_DEG)
    n_seeds = len(seeds)

    # Build per-scenario/per-candidate stats
    print("\n" + "=" * 80)
    print(f"多种子 MC 汇总 (mean +/- std RMSE, km, n={n_seeds} seeds)")
    print("=" * 80)

    header = f"{'Scenario \\ rate':<18}"
    for ci in range(n_cand):
        header += f" | {CANDIDATES_DEG[ci]:6.1f} deg/s "
    header += f" | {'Default':>8} | {'Oracle':>8} | {'MM-upper':>8}"
    print(header)
    print("-" * 90)

    default_all = []
    oracle_all = []
    mm_all = []

    for sci in range(n_sc):
        sname = SCENARIOS[sci]
        line = f"{sname:<18}"

        # Collect values per candidate across seeds
        cand_values = [[] for _ in range(n_cand)]
        for (seed, sc, ci), val in done_map.items():
            if sc == sci + 1 and val is not None and math.isfinite(val):
                si = seeds.index(seed) if seed in seeds else -1
                if si >= 0:
                    cand_values[ci-1].append(val)

        for ci in range(n_cand):
            vals = cand_values[ci]
            if vals:
                mean_v = sum(vals)/len(vals)
                std_v = (sum((x-mean_v)**2 for x in vals)/max(len(vals)-1,1))**0.5 if len(vals)>1 else 0
                line += f" | {mean_v:+.2f}{std_v:+.2f}"
            else:
                line += " |    nan"

        # Default (1.0 deg/s)
        def_idx = next((i for i, c in enumerate(CANDIDATES_DEG) if abs(c - 1.0) < 1e-6), -1)
        if def_idx >= 0:
            d_vals = [v for (s, sc, ci), v in done_map.items() if sc==sci+1 and ci==def_idx+1 and math.isfinite(v)]
        else:
            d_vals = []

        # Oracle (true rate)
        orc_idx = next((i for i, c in enumerate(CANDIDATES_DEG) if abs(c - TRUE_RATES[sci]) < 1e-6), -1)
        if orc_idx >= 0:
            o_vals = [v for (s, sc, ci), v in done_map.items() if sc==sci+1 and ci==orc_idx+1 and math.isfinite(v)]
        else:
            o_vals = []

        # MM upper (min per seed)
        # Group by seed
        seed_vals = {}
        for (seed, sc, ci), val in done_map.items():
            if sc == sci + 1 and math.isfinite(val):
                if seed not in seed_vals:
                    seed_vals[seed] = []
                seed_vals[seed].append(val)
        mm_vals = [min(v) for v in seed_vals.values()]

        d_mean = sum(d_vals)/len(d_vals) if d_vals else float('nan')
        d_std = (sum((x-d_mean)**2 for x in d_vals)/max(len(d_vals)-1,1))**0.5 if len(d_vals)>1 else 0
        o_mean = sum(o_vals)/len(o_vals) if o_vals else float('nan')
        o_std = (sum((x-o_mean)**2 for x in o_vals)/max(len(o_vals)-1,1))**0.5 if len(o_vals)>1 else 0
        m_mean = sum(mm_vals)/len(mm_vals) if mm_vals else float('nan')
        m_std = (sum((x-m_mean)**2 for x in mm_vals)/max(len(mm_vals)-1,1))**0.5 if len(mm_vals)>1 else 0

        line += f" | {d_mean:8.2f}±{d_std:.2f}"
        line += f" | {o_mean:8.2f}±{o_std:.2f}"
        line += f" | {m_mean:8.2f}±{m_std:.2f}"
        print(line)

        default_all.extend(d_vals)
        oracle_all.extend(o_vals)
        mm_all.extend(mm_vals)

    # OVERALL
    print(f"{'OVERALL(pooled)':<18}", end='')
    pooled_means = []
    for ci in range(n_cand):
        vals = [v for (s, sc, c), v in done_map.items() if c==ci+1 and math.isfinite(v)]
        if vals:
            pooled_means.append(sum(vals)/len(vals))
            pooled_std = (sum((x-sum(vals)/len(vals))**2 for x in vals)/max(len(vals)-1,1))**0.5 if len(vals)>1 else 0
            print(f" | {pooled_means[-1]:8.2f}±{pooled_std:.2f}", end='')
    if default_all:
        dm = sum(default_all)/len(default_all)
        ds = (sum((x-dm)**2 for x in default_all)/max(len(default_all)-1,1))**0.5 if len(default_all)>1 else 0
        print(f" | {dm:8.2f}±{ds:.2f}", end='')
    if oracle_all:
        om = sum(oracle_all)/len(oracle_all)
        osv = (sum((x-om)**2 for x in oracle_all)/max(len(oracle_all)-1,1))**0.5 if len(oracle_all)>1 else 0
        print(f" | {om:8.2f}±{osv:.2f}", end='')
    if mm_all:
        mmm = sum(mm_all)/len(mm_all)
        ms = (sum((x-mmm)**2 for x in mm_all)/max(len(mm_all)-1,1))**0.5 if len(mm_all)>1 else 0
        print(f" | {mmm:8.2f}±{ms:.2f}")
    print()

    # Improvement
    print(f"\n============================================================")
    print(f"改善 vs Default (pooled, {n_seeds} seeds x {n_sc} scenes)")
    print(f"============================================================")
    if default_all:
        d_mean = sum(default_all)/len(default_all)
        o_mean = sum(oracle_all)/len(oracle_all) if oracle_all else 0
        m_mean = sum(mm_all)/len(mm_all) if mm_all else 0
        print(f"  Default(1 deg/s) : {d_mean:.3f} km  (baseline)")
        print(f"  Oracle(真实)     : {o_mean:.3f} km  (improve {((d_mean-o_mean)/d_mean*100):+.1f}%)")
        print(f"  MM-upper(min)    : {m_mean:.3f} km  (improve {((d_mean-m_mean)/d_mean*100):+.1f}%)")

    print()


def main():
    seed_start = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    seed_end = int(sys.argv[2]) if len(sys.argv) > 2 else 50
    seeds = list(range(seed_start, seed_end + 1))

    done_map = load_progress()

    total_tasks = len(seeds) * len(SCENARIOS) * len(CANDIDATES_DEG)
    pending = [(s, sc, ci, CANDIDATES_DEG[ci-1], SCENARIOS[sc-1])
               for s in seeds for sc in range(1, len(SCENARIOS)+1) for ci in range(1, len(CANDIDATES_DEG)+1)
               if (s, sc, ci) not in done_map]

    print("=" * 60)
    print("turn_rate 多种子 MC 扫描 (Python scheduler)")
    print(f"Seeds: {len(seeds)} [{seeds[0]}..{seeds[-1]}]")
    print(f"Candidates: {CANDIDATES_DEG}")
    print(f"Scenarios: {SCENARIOS}")
    print(f"Total: {total_tasks}, Pending: {len(pending)}, Done: {len(done_map)}")
    print("=" * 60)

    if not pending:
        print("\nAll tasks already completed!")
        print_summary(done_map, seeds)
        return

    start_time = time.time()

    for idx, (seed, sci, ci, rate, sname) in enumerate(pending):
        result = run_single(seed, sname, rate)

        key = (seed, sci, ci)
        if result not in (None, 'timeout', 'error') and isinstance(result, (int, float)) and not math.isnan(result):
                save_done(key, result)
                elapsed = time.time() - start_time
                eta = elapsed / (idx + 1) * (len(pending) - idx - 1)
                print(f"  [{idx+1}/{len(pending)}] seed={seed} {sname} rate={rate}deg RMSE={result:.3f}km elapsed={elapsed:.0f}s ETA={eta:.0f}s")

    # Summary
    elapsed_total = time.time() - start_time
    print(f"\nDone! Total wall time: {elapsed_total:.0f}s ({elapsed_total/max(len(pending),1):.1f}s/task)")
    print_summary(done_map, seeds)


if __name__ == "__main__":
    import math
    main()
