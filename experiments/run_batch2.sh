#!/bin/bash
# 分批实验：每个种子一个独立 MATLAB 进程，避免长时间运行导致的内存/进程崩溃
# 用法: bash run_batch2.sh [exp_name]
#   不传参数则依次跑 E1_small / E6_kappa / E7_model

cd "D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn/experiments" || exit 1
MATLAB="C:/Program Files/MATLAB/R2023a/bin/matlab"

run_one() {
  local exp=$1
  local seed=$2
  echo "=== $exp seed $seed  $(date +%H:%M:%S) ==="
  "$MATLAB" -batch "run_experiments('$exp', $seed)" >> "batch_${exp}.log" 2>&1
  local rc=$?
  echo "    exit=$rc  $(date +%H:%M:%S)"
}

EXPS="${1:-E1_small E6_kappa E7_model}"

for exp in $EXPS; do
  echo ""
  echo "########## $exp ##########"
  for s in 1 2 3 4 5 6 7 8 9 10; do
    run_one "$exp" "$s"
  done
done

echo ""
echo "########## ALL DONE $(date +%H:%M:%S) ##########"
