#!/bin/bash
cd "d:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn" || exit 1
CSV=results/seed_sweep/adsb_ns_all_seeds.csv
for s in "$@"; do
  if grep -q "^$s," "$CSV" 2>/dev/null; then echo "seed $s 已有结果，跳过"; continue; fi
  for attempt in 1 2 3; do
    echo "== seed $s 第 $attempt 次 =="
    "C:/Program Files/MATLAB/R2023a/bin/matlab" -batch "addpath(pwd); addpath('config'); addpath('simulation'); addpath('utils'); addpath('ukf'); addpath('tracker'); addpath('fusion'); addpath('evaluation'); run_seed_sweep('adsb_ns_all', $s);" > results/seed_sweep/log_$s.txt 2>&1
    code=$?
    if grep -q "^$s," "$CSV" 2>/dev/null; then echo "seed $s OK (exit $code)"; break; fi
    echo "seed $s 失败 exit=$code"
  done
done
tail -3 "$CSV"
