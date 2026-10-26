function S = load_aligned_snapshots(scenario_name, params_override)
% load_aligned_snapshots — 调用 run() 的 fast_mode，返回对齐后的双站航迹快照
%
% run() 内部所有阶段函数（calibrate_bias / generate_radar_detections /
% run_oracle_tracker / time_align_tracks 等）均为 run.m 局部函数，必须从 run()
% 内部调用；本函数通过 fast_mode 让 run() 跳过融合/绘图，仅产出并交还对齐快照。
% 返回结构体 S 含 trackSnapshots_R1 / aligned_R2 / truthTrajs / scenario /
%   params / n_frames / truth_all，可直接喂给 compare_pairing_methods。
    if nargin < 1 || isempty(scenario_name), scenario_name = 'adsb_0002'; end
    repo_root = fileparts(fileparts(mfilename('fullpath')));   % _audit -> 仓库根
    code_dirs = {'config','simulation','ukf','tracker','fusion','evaluation', ...
                 'utils','visualization','validation','experiments','io'};
    addpath(repo_root);
    for cd_i = 1:numel(code_dirs)
        d_i = fullfile(repo_root, code_dirs{cd_i});
        if exist(d_i, 'dir'), addpath(d_i); end
    end

    if nargin < 2 || ~isstruct(params_override)
        params = simulation_params_oracle();
    else
        params = params_override;
    end
    params.fast_mode = true;   % 跳过融合/绘图，仅返回对齐快照

    result = run(scenario_name, params);   % run() 内部阶段函数可见
    S = result;                            % 已含所需全部字段
    [n_tb, ~, ~, ~, ~] = pairing.build_truth_pairs(S.trackSnapshots_R1, S.aligned_R2);
    fprintf('[loader] %s 完成: %d 帧, 两站共现真实目标=%d\n', scenario_name, S.n_frames, n_tb);
end
