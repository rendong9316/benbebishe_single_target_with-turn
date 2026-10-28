function s = detection_rng(action, params, radar_id)
%DETECTION_RNG 点迹生成的"每站一条"独立随机流
%
%   detection_rng('init', params)        % 重置两条流（每次 run 开始时调用一次）
%   rs = detection_rng('get', params, r) % 取第 r 站（1/2）的 RandStream
%
%   【为什么要单独管这个流】
%   1) 两站量测必须相互独立。若两站共用一条全局流，R2 的检测数变化会平移 R1 的
%      随机序列，两站噪声就不再独立采样；场景里目标增减也会悄悄改变另一站的噪声。
%   2) 点迹噪声不应受其它模块的随机消耗影响。Stage 1 的 calibrate_radar_bias 内部
%      会 rng(params.random_seed) 并抽 randn 做标校侧量测，它跑在逐帧循环之前，
%      会把全局流"吃掉"一截——于是同一 seed 下，改配准参数就会改变点迹。
%      这会让"复现"变成空话（旧在线链路 run_online.m 是 rng(seed+1e7)/rng(seed+2e7)
%      两站各自成流，重构后丢掉，是 2026-10-28 复核发现的问题）。
%   3) 显式成流后，"同 seed 同场景 → 同点迹"与配准模式、跟踪器实现彻底解耦。
%
%   与旧链路的对应关系：旧 run_online.m 的 generate_detections 用
%   rng(seed+1e7)（R1）/ rng(seed+2e7)（R2），本函数用同两个种子建 mt19937ar 流，
%   因此逐帧调用顺序相同的条件下，抽样序列与旧链路逐位一致。
%
%   用法（在点迹生成里）：
%       rs = detection_rng('get', params, radar_id);
%       prev = RandStream.getGlobalStream();
%       RandStream.setGlobalStream(rs);
%       restore = onCleanup(@() RandStream.setGlobalStream(prev));
%
%   见 docs/收尾报告复核_20261028.md §3.3。

    persistent RS
    if nargin < 3, radar_id = 0; end
    if strcmpi(action, 'init') || isempty(RS)
        RS = struct();
        RS.rs1 = RandStream('mt19937ar', 'Seed', params.random_seed + 1e7);
        RS.rs2 = RandStream('mt19937ar', 'Seed', params.random_seed + 2e7);
    end
    if strcmpi(action, 'init')
        s = [];
        return;
    end
    if radar_id == 1
        s = RS.rs1;
    else
        s = RS.rs2;
    end
end
