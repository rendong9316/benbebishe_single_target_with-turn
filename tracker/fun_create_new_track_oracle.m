function newTrack = fun_create_new_track_oracle(det1, det2, ukf_tpl, params, frame_id, next_id, truth_idx, real_hist)
    % Oracle 模式新航迹创建函数
    %
    % 输入：
    %   det1     — 最早有效检测（滑窗中的第一条）
    %   det2     — 当前帧检测（滑窗中的最后一条，即确认帧）
    %   real_hist — 滑窗中所有真实检测的历史（数量达到配置确认阈值）
    %
    % 创建流程：
    %   1. 验证 real_hist 的有效性（帧号递增、在窗口内、truth 一致）
    %   2. 用 det1+det2 两点法初始化 UKF（位置+速度）
    %   3. 组装航迹结构体（南阳式字段：Quality, type, asscPointList 等）
    %   4. 设置初始状态：RELIABLE_TRACK, Quality=confirm_quality

    % ---- 历史有效性校验 ----
    % 真实检测数量必须达到配置的确认阈值
    % 如果 real_hist 为空或检测数不足，说明滑窗中真实命中不够，不能起始
    if nargin < 8 || isempty(real_hist) || length(real_hist) < params.oracle_QUALIFY_NUM
        error('fun_create_new_track_oracle:invalidHistory', ...
            '创建可靠航迹需要达到配置确认阈值的实际检测历史');
    end

    % 检查帧号严格递增、以当前帧结束、窗口跨度 <= TOLERANT_NUM
    % 防止跨度过大的虚假检测被用于航迹起始
    history_frames = [real_hist.frameID];
    if any(diff(history_frames) <= 0) || history_frames(end) ~= frame_id || ...
            frame_id - history_frames(1) + 1 > params.oracle_TOLERANT_NUM
        error('fun_create_new_track_oracle:invalidHistory', ...
            '起始检测历史必须按帧递增、以确认帧结束且位于配置窗口内');
    end

    % 逐一验证每个历史点迹：非空、非杂波、aircraft_id 匹配 truth_idx
    % 确保滑窗中的所有检测都来自同一个真实目标，排除杂波混入
    for i = 1:length(real_hist)
        dp = real_hist(i).point;
        if isempty(dp) || ~isstruct(dp) || ~isfield(dp, 'aircraft_id') || ...
                double(dp.aircraft_id) ~= double(truth_idx) || ...
                (isfield(dp, 'is_clutter') && dp.is_clutter) || ...
                double(dp.frameID) ~= double(real_hist(i).frameID)
            error('fun_create_new_track_oracle:invalidHistory', ...
                '起始历史包含无效、虚警或 truth 不匹配的检测');
        end
    end

    % ---- UKF 初始化：两点法 ----
    % 用最早检测(det1)和当前检测(det2)的差分计算初始速度
    % det1 和 det2 的时间差决定了速度估计的精度
    % ukf_dispatch('init') 内部完成 Sigma 点采样和初始协方差设置
    new_ukf = ukf_dispatch('init', ukf_tpl, det1, det2);

    % 后初始化：注入 dt、初始化标志、NIS 历史、Q_ema 等
    % post_init_multi 会在 UKF 结构体中添加必要的元数据字段
    new_ukf = post_init_multi(new_ukf, params);

    % ---- 输出点：当前帧的平滑输出（用 UKF 估计位置替代原始检测） ----
    % make_output_point 将 UKF 滤波后的经纬度写入输出点结构体，
    % 替代原始检测的位置，使平滑输出更准确
    smoothPoint = make_output_point(det2, new_ukf, frame_id);

    % ---- 组装关联点迹历史 ----
    % 将滑窗中所有真实检测按顺序存入 asscPointList，
    % 后续可用于航迹质量评估和诊断分析
    asscPointList = cell(1, length(real_hist));
    for i = 1:length(real_hist)
        asscPointList{i} = real_hist(i).point;
    end

    % ---- 航迹结构体组装 ----
    % 计算航迹的关键统计量：出生帧、确认帧、跨度、关联点数
    birth_frame = double(real_hist(1).frameID);   % 第一条检测的帧号
    confirm_frame = frame_id;                      % 确认帧号
    span = confirm_frame - birth_frame + 1;        % 航迹跨度（帧数）
    assoc_count = length(real_hist);               % 关联点迹数

    newTrack = struct();
    newTrack.id = next_id;                         % 全局递增航迹 ID
    newTrack.truth_idx = truth_idx;                % 关联的真值目标编号
    % 南阳式航迹类型：RELIABLE_TRACK = 1（直接确认为可靠航迹）
    newTrack.Type = params.RELIABLE_TRACK;
    newTrack.type = params.RELIABLE_TRACK;
    newTrack.Quality = params.oracle_confirm_quality;  % 初始质量分（满分）
    newTrack.quality = params.oracle_confirm_quality;
    newTrack.isNewTrack = 1;                       % 新航迹标记（下一帧清除）
    newTrack.updateFlag = 1;                       % 本帧已更新
    newTrack.TotalPointCnt = span;                 % 总存活帧数
    newTrack.AsscPointCnt = assoc_count;           % 关联点迹数
    newTrack.TotalLostPointCnt = span - assoc_count; % 总漏检数
    newTrack.SuccLossPointCnt = 0;                 % 连续漏检数（初始为 0）
    newTrack.missed = 0;                           % 当前是否漏检
    newTrack.asscPointList = asscPointList;        % 关联点迹历史
    newTrack.predictRes = {};                      % 预测结果（预留）
    newTrack.smoothPointList = {smoothPoint};      % 平滑输出点历史
    newTrack.outputPointList = newTrack.smoothPointList;  % 对外输出点
    newTrack.ukf = new_ukf;                        % UKF 滤波器状态
    newTrack.lat = new_ukf.x(3);                   % 纬度（从 UKF 状态提取）
    newTrack.lon = new_ukf.x(1);                   % 经度
    newTrack.life = span;                          % 航迹寿命（帧数）
    newTrack.birth_frame = birth_frame;            % 出生帧
    newTrack.confirm_frame = confirm_frame;        % 确认帧
    newTrack.death_frame = [];                     % 死亡帧（初始为空）
    newTrack.death_reason = '';                    % 死亡原因
    newTrack.BatchNo = [];                         % 批次号（预留）
    newTrack.assoc_det = det2;                     % 当前关联点迹
    newTrack.nis_history = [];                     % NIS 历史（初始为空）
end

function p = make_output_point(det, ukf, frame_id)
    % 制作输出点：复用检测结构体，将位置替换为 UKF 平滑估计值
    % 这样平滑输出的位置比原始检测更精确
    p = det;
    p.frameID = frame_id;
    p.lon = ukf.x(1);
    p.lat = ukf.x(3);
end
