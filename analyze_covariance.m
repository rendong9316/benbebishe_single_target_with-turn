% 分析协方差估计和实际误差的关系
addpath(genpath('D:\Desktop\single_target_with-turn'));
data = load('D:\Desktop\single_target_with-turn\results\simulation_multi_20260702_201248.mat');

% 提取变量
trackSnapshots_R1 = data.trackSnapshots_R1;
aligned_R2 = data.aligned_R2;
truthTrajs = data.truthTrajs;

n_frames = length(trackSnapshots_R1);

fprintf('总帧数: %d\n', n_frames);
fprintf('真值目标数: %d\n', length(truthTrajs));

% 查看真值轨迹的结构
fprintf('\n真值轨迹字段:\n');
disp(fieldnames(truthTrajs(1)));

% 计算每个目标的实际误差和协方差估计
fprintf('\n=== R1 协方差估计 vs 实际误差 ===\n');
for ac = 1:3
    % 找R1中ac_idx=ac的航迹
    r1_id = [];
    for k = 1:n_frames
        trks = trackSnapshots_R1{k}.trackList;
        for t = 1:length(trks)
            if trks{t}.type ~= 7 && isfield(trks{t}, 'ac_idx') && trks{t}.ac_idx == ac
                r1_id = trks{t}.id;
                break;
            end
        end
        if ~isempty(r1_id), break; end
    end
    
    if isempty(r1_id), continue; end
    
    % 收集误差和协方差
    pos_errors = [];
    cov_traces = [];
    
    for k = 1:n_frames
        trks = trackSnapshots_R1{k}.trackList;
        for t = 1:length(trks)
            if trks{t}.id == r1_id && trks{t}.type ~= 7 && ~isnan(trks{t}.lat)
                % 计算位置误差
                truth_lon = truthTrajs(ac).lon(k);
                truth_lat = truthTrajs(ac).lat(k);
                dist = sphere_utils_haversine_distance(trks{t}.lon, trks{t}.lat, truth_lon, truth_lat) / 1000;
                pos_errors(end+1) = dist;
                
                % 协方差迹
                if ~isempty(trks{t}.ukf) && isfield(trks{t}.ukf, 'P')
                    cov_traces(end+1) = trace(trks{t}.ukf.P);
                end
                break;
            end
        end
    end
    
    fprintf('目标 %d (R1#%d):\n', ac, r1_id);
    fprintf('  实际位置误差 - 均值: %.2f km, 中位数: %.2f km, RMSE: %.2f km\n', ...
        mean(pos_errors), median(pos_errors), sqrt(mean(pos_errors.^2)));
    fprintf('  协方差迹 - 均值: %.6f, 中位数: %.6f\n', ...
        mean(cov_traces), median(cov_traces));
end

fprintf('\n=== R2 协方差估计 vs 实际误差 ===\n');
for ac = 1:3
    % 找R2中ac_idx=ac的航迹
    r2_id = [];
    for k = 1:n_frames
        trks = aligned_R2{k}.trackList;
        for t = 1:length(trks)
            if trks{t}.type ~= 7 && isfield(trks{t}, 'ac_idx') && trks{t}.ac_idx == ac
                r2_id = trks{t}.id;
                break;
            end
        end
        if ~isempty(r2_id), break; end
    end
    
    if isempty(r2_id), continue; end
    
    % 收集误差和协方差
    pos_errors = [];
    cov_traces = [];
    
    for k = 1:n_frames
        trks = aligned_R2{k}.trackList;
        for t = 1:length(trks)
            if trks{t}.id == r2_id && trks{t}.type ~= 7 && ~isnan(trks{t}.lat)
                % 计算位置误差
                truth_lon = truthTrajs(ac).lon(k);
                truth_lat = truthTrajs(ac).lat(k);
                dist = sphere_utils_haversine_distance(trks{t}.lon, trks{t}.lat, truth_lon, truth_lat) / 1000;
                pos_errors(end+1) = dist;
                
                % 协方差迹
                if ~isempty(trks{t}.ukf) && isfield(trks{t}.ukf, 'P')
                    cov_traces(end+1) = trace(trks{t}.ukf.P);
                end
                break;
            end
        end
    end
    
    fprintf('目标 %d (R2#%d):\n', ac, r2_id);
    fprintf('  实际位置误差 - 均值: %.2f km, 中位数: %.2f km, RMSE: %.2f km\n', ...
        mean(pos_errors), median(pos_errors), sqrt(mean(pos_errors.^2)));
    fprintf('  协方差迹 - 均值: %.6f, 中位数: %.6f\n', ...
        mean(cov_traces), median(cov_traces));
end
