#!/bin/bash
# 并行实验运行器：按 job 列表并发跑，K 个 MATLAB 进程同时工作
#
# job 列表格式（每行）：exp_id|seed|scene_filter
#   例：E13_dense|2|adsb_0608
#
# 用法：bash run_par.sh jobs.txt [K]
#   默认 K=6（16 核机器上留余量）

cd "D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn/experiments" || exit 1
MATLAB="C:/Program Files/MATLAB/R2023a/bin/matlab"
JOBFILE="${1:?need job file}"
K="${2:-6}"

if [ ! -f "$JOBFILE" ]; then echo "job file not found: $JOBFILE"; exit 1; fi
NJOB=$(grep -c . "$JOBFILE")
echo "########## PAR START $(date +%H:%M:%S)  jobs=$NJOB  K=$K ##########"

run_job() {
  local line="$1"
  IFS='|' read -r exp sd sc <<< "$line"
  local tag="${exp}_${sc:-all}_s${sd}"
  if [ -n "$sc" ]; then
    "$MATLAB" -batch "run_experiments('$exp', $sd, {'$sc'})" >> "par_${exp}.log" 2>&1
  else
    "$MATLAB" -batch "run_experiments('$exp', $sd)" >> "par_${exp}.log" 2>&1
  fi
  echo "  DONE $tag  exit=$?  $(date +%H:%M:%S)"
}

i=0
while IFS= read -r line; do
  [ -z "$line" ] && continue
  run_job "$line" &
  i=$((i+1))
  if [ $((i % K)) -eq 0 ]; then wait; fi
done < "$JOBFILE"
wait

echo "########## PAR DONE $(date +%H:%M:%S) ##########"
