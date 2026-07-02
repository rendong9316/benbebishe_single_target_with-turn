% 简化版：分析协方差估计和实际误差的关系
addpath(genpath('D:\Desktop\single_target_with-turn'));
data = load('D:\Desktop\single_target_with-turn\results\simulation_multi_20260702_201248.mat');

trackSnapshots_R1 = data.trackSnapshots_R1;
aligned_R2 = data.aligned_R2;
truthTrajs = data.truthTrajs;

n_frames = length(trackSnapshots_R1);

% 直接用已有的errorStats
fprintf('=== R1 误差统计 (来自 errorStats_R1) ===\n');
disp(data.errorStats_R1);

fprintf('\n=== R2 误差统计 (来自 errorStats_R2) ===\n');
disp(data.errorStats_R2);

fprintf('\n=== 融合误差统计 (来自 fusion_eval) ===\n');
disp(data.fusion_eval);

% 简单计算一下R1和R2的协方差迹
fprintf('\n=== 协方差迹统计 ===\n');
r1_traces = [];
r2_traces = [];

for k = 1:n_frames
    trks1 = trackSnapshots_R1{k}.trackList;
    for t = 1:length(trks1)
        if trks1{t}.type ~= 7 && ~isempty(trks1{t}.ukf) && isfield(trks1{t}.ukf, 'P')
            r1_traces(end+1) = trace(trks1{t}.ukf.P);
        end
    end
    
    trks2 = aligned_R2{k}.trackList;
    for t = 1:length(trks2)
        if trks2{t}.type ~= 7 && ~isempty(trks2{t}.ukf) && isfield(trks2{t}.ukf, 'P')
            r2_traces(end+1) = trace(trks2{t}.ukf.P);
        end
    end
end

fprintf('R1协方差迹: 均值=%.6f, 中位数=%.6f\n', mean(r1_traces), median(r1_traces));
fprintf('R2协方差迹: 均值=%.6f, 中位数=%.6f\n', mean(r2_traces), median(r2_traces));
fprintf('R1/R2迹比值: %.2f\n', mean(r1_traces)/mean(r2_traces));
