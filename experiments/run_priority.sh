#!/bin/bash
# 优先级批：E6 只跑 seed1（验证 CCF 无效），E7 跑 seed1-5（关键：模型集对 rho 的影响）
cd "D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn/experiments" || exit 1
MATLAB="C:/Program Files/MATLAB/R2023a/bin/matlab"

echo "=== E6_kappa seed1 (CCF 验证) $(date +%H:%M:%S) ==="
"$MATLAB" -batch "run_experiments('E6_kappa', 1)" >> batch_E6_kappa.log 2>&1
echo "    exit=$? $(date +%H:%M:%S)"

for s in 1 2 3 4 5; do
  echo "=== E7_model seed$s (模型集关键实验) $(date +%H:%M:%S) ==="
  "$MATLAB" -batch "run_experiments('E7_model', $s)" >> batch_E7_model.log 2>&1
  echo "    exit=$? $(date +%H:%M:%S)"
done

echo "=== DONE $(date +%H:%M:%S) ==="
