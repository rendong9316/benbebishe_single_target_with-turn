#!/bin/bash
# 第二批大规模 MC：
#   1) E12_sweep   模型集细扫 6 档 × 15 种子（找最优档位，验证"匹配"而非"越小越好"）
#   2) E11_mid     中间档 0.2/0.7 × 50 种子（验证 rho 单调性）
#   3) E9/E10      基线/最优各补到 100 种子（种子 51-100）
# 用法: bash run_mc2.sh [--no-extend]
cd "D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn/experiments" || exit 1
MATLAB="C:/Program Files/MATLAB/R2023a/bin/matlab"

echo "########## MC2 START $(date +%H:%M:%S) ##########"

echo ""
echo "===== E12_sweep (6 configs x 15 seeds) ====="
for s in $(seq 1 15); do
  "$MATLAB" -batch "run_experiments('E12_sweep', $s)" >> "mc_E12_sweep.log" 2>&1
  echo "  E12 seed $s  exit=$?  $(date +%H:%M:%S)"
done

echo ""
echo "===== E11_mid (0.2/0.7 x 50 seeds) ====="
for s in $(seq 1 50); do
  "$MATLAB" -batch "run_experiments('E11_mid', $s)" >> "mc_E11_mid.log" 2>&1
  echo "  E11 seed $s  exit=$?  $(date +%H:%M:%S)"
done

if [ "$1" != "--no-extend" ]; then
  echo ""
  echo "===== EXTEND E9/E10 to 100 seeds (51-100) ====="
  for exp in E9_baseline E10_optimal; do
    for s in $(seq 51 100); do
      "$MATLAB" -batch "run_experiments('$exp', $s)" >> "mc_${exp}.log" 2>&1
      echo "  $exp seed $s  exit=$?  $(date +%H:%M:%S)"
    done
  done
fi

echo ""
echo "########## MC2 DONE $(date +%H:%M:%S) ##########"
