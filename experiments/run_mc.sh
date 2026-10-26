#!/bin/bash
# 大规模蒙特卡洛：基线(0.7/1.5) / 中档(0.2/0.7) / 最优(0.05/0.2)，种子 1-50
# 已有结果会自动跳过（E7 有 1-5，E8 有 6-15 的最优档）
# 用法: bash run_mc.sh [start_seed] [end_seed]
cd "D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn/experiments" || exit 1
MATLAB="C:/Program Files/MATLAB/R2023a/bin/matlab"
S0="${1:-1}"
S1="${2:-50}"

echo "########## MC START $(date +%H:%M:%S)  seeds $S0-$S1 ##########"

for exp in E9_baseline E10_optimal; do
  echo ""
  echo "===== $exp ====="
  for s in $(seq $S0 $S1); do
    "$MATLAB" -batch "run_experiments('$exp', $s)" >> "mc_${exp}.log" 2>&1
    echo "  $exp seed $s  exit=$?  $(date +%H:%M:%S)"
  done
done

echo ""
echo "########## MC DONE $(date +%H:%M:%S) ##########"
