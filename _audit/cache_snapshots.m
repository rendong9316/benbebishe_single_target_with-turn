function cache_snapshots(plan_name)
% cache_snapshots — 一次性缓存「对齐快照」的精简版，供后续调参/对比直接 load
%
% 【为什么】单场景耗时 99% 在跟踪（adsb_0608 47s / adsb_0911 78s），配对本身
%   <1s。调参实验若每次重跑跟踪，纯属浪费。缓存后每次只需 load + 配对（几秒）。
%
% 【精简策略】只保留配对与评估真正用到的字段，并剔除历史航迹（type==7）：
%   - 配对：id / type / lat / lon / ukf.x / ukf.P
%   - 评估：truth_idx（eval_pairing 判对错用）
%   历史航迹 build_track_index 本来就会跳过，剔除后结果完全不变。
%   体积从 261MB（含全量航迹+冗余字段）降到 ~2-3MB/场景。
%
% 用法: cache_snapshots()         % 全部 13 组（稀疏 8 + 密集 5）
%       cache_snapshots('dense')  % 仅密集 5 组
%       cache_snapshots('sparse') % 仅稀疏 8 组
    if nargin < 1 || isempty(plan_name), plan_name = 'all'; end
    repo_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(repo_root);
    code_dirs = {'config','simulation','ukf','tracker','fusion','evaluation', ...
                 'utils','visualization','validation','experiments','io'};
    for cd_i = 1:numel(code_dirs)
        d_i = fullfile(repo_root, code_dirs{cd_i});
        if exist(d_i, 'dir'), addpath(d_i); end
    end

    cache_dir = fullfile(repo_root, '_audit', 'cache');
    if ~exist(cache_dir, 'dir'), mkdir(cache_dir); end

    all_plan = {...
        'adsb_0002', 101; 'adsb_0002', 102; 'adsb_0002', 103; ...
        'adsb_0002', 104; 'adsb_0002', 105; ...
        'adsb_span', 201; 'adsb_span', 202; 'adsb_span', 203; ...
        'adsb_0608', 301; 'adsb_0608', 302; 'adsb_0608', 303; ...
        'adsb_0911', 401; 'adsb_0911', 402};

    switch lower(plan_name)
        case 'dense',  plan = all_plan(9:13, :);
        case 'sparse', plan = all_plan(1:8, :);
        otherwise,     plan = all_plan;
    end

    for r = 1:size(plan, 1)
        scene = plan{r, 1};
        seed  = plan{r, 2};
        out = fullfile(cache_dir, sprintf('%s_%d.mat', scene, seed));
        if isfile(out)
            fprintf('[skip] %s 已存在\n', out);
            continue;
        end
        params = simulation_params_oracle();
        params.random_seed = seed;
        params.imm_ct_omega_slow_deg = 0.05;
        params.imm_ct_omega_fast_deg = 0.2;

        fprintf('\n[cache] %s seed %d ...\n', scene, seed);
        tic; S = load_aligned_snapshots(scene, params); t_load = toc;
        S2 = slim_S(S, scene, seed);
        save(out, '-struct', 'S2', '-v7.3');
        d = dir(out);
        fprintf('[cache] %s seed %d 完成: 跟踪 %.1fs -> 缓存 %.1f MB\n', ...
            scene, seed, t_load, d.bytes / 1e6);
    end
    fprintf('\n全部缓存位于: %s\n', cache_dir);
end


function S2 = slim_S(S, scene, seed)
    S2 = struct();
    S2.scene = scene;
    S2.seed = seed;
    S2.params = S.params;
    S2.n_frames = S.n_frames;
    S2.trackSnapshots_R1 = slim_snaps(S.trackSnapshots_R1);
    S2.aligned_R2 = slim_snaps(S.aligned_R2);
    S2.truthTrajs = S.truthTrajs;
end


function out = slim_snaps(snaps)
% 只保留活跃航迹（type~=7 且位置有效）与必要字段
    nf = numel(snaps);
    out = cell(nf, 1);
    for k = 1:nf
        if ~isfield(snaps{k}, 'trackList') || isempty(snaps{k}.trackList)
            out{k} = struct('trackList', {{}});
            continue;
        end
        trks = snaps{k}.trackList;
        keep = cell(1, numel(trks));
        n = 0;
        for t = 1:numel(trks)
            trk = trks{t};
            if trk.type == 7, continue; end                       % 历史航迹不入缓存
            if ~isfield(trk, 'lat') || isnan(trk.lat), continue; end
            n = n + 1;
            s = struct();
            s.id  = trk.id;
            s.type = trk.type;
            % life：dualgate 的 compute_max_life 用它做 T1 质量修正，必须保留
            if isfield(trk, 'life'), s.life = trk.life; else, s.life = 0; end
            s.lat = trk.lat;
            s.lon = trk.lon;
            if isfield(trk, 'truth_idx'), s.truth_idx = trk.truth_idx; else, s.truth_idx = 0; end
            if isfield(trk, 'ukf') && ~isempty(trk.ukf) && isfield(trk.ukf, 'x') && isfield(trk.ukf, 'P')
                s.ukf = struct('x', trk.ukf.x, 'P', trk.ukf.P);
                % Q 必须保留：融合下游 build_faded_track_segments 需要 trk.ukf.Q
                % （缺失会报「无法识别的字段名称 "Q"」）。配对本身不用，但
                % 缓存要同时支撑「配对质量」与「融合 RMSE」两类下游实验。
                if isfield(trk.ukf, 'Q'), s.ukf.Q = trk.ukf.Q; end
            else
                s.ukf = [];
            end
            keep{n} = s;
        end
        out{k} = struct();
        out{k}.trackList = keep(1:n);
    end
end
