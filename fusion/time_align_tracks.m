% =========================================================================
% time_align_tracks.m
% 时间对齐: 将异步采样的雷达航迹外推到统一时间基准
% =========================================================================
% 问题: R1采样于0s/30s/60s, R2采样于13s/43s/73s, 相差13s
% 方案: 以R1时间网格为基准, 将R2 UKF状态用CV模型回退13s
%       x(t-Δt) = F(-Δt)·x(t),  P(t-Δt) = F·P·F' + Q(|Δt|)
% =========================================================================

function aligned_R2 = time_align_tracks(trackSnapshots_R2, params)
    dt_offset = params.time_offset_radar2_sec;  % R2滞后R1的秒数 (13s)

    n_frames = length(trackSnapshots_R2);
    aligned_R2 = cell(n_frames, 1);

    for k = 1:n_frames
        snap = trackSnapshots_R2{k};
        aligned_snap = snap;

        for t = 1:length(snap.trackList)
            trk = snap.trackList{t};
            if trk.type == 7, continue; end  % 跳过已终止航迹
            if trk.type == 6, continue; end  % 跳过未起始的临时航迹
            if isempty(trk.ukf) || ~isfield(trk.ukf, 'x') || isempty(trk.ukf.x)
                continue;
            end

            % CV模型状态转移矩阵 (Δt = -offset, 回退)
            dt = -dt_offset;
            F = [1, dt, 0, 0;
                 0,  1, 0, 0;
                 0,  0, 1, dt;
                 0,  0, 0,  1];

            % 状态回退
            trk.ukf.x = F * trk.ukf.x;

            % 协方差传播 (用|dt|做过程噪声)
            dt_abs = abs(dt);
            Q_dt = trk.ukf.Q * (dt_abs / params.dt_sec);
            trk.ukf.P = F * trk.ukf.P * F' + Q_dt;
            trk.ukf.P = regularize_cov(trk.ukf.P);

            % 更新位置
            trk.lat = trk.ukf.x(3);
            trk.lon = trk.ukf.x(1);

            % 标记已对齐
            trk.time_aligned = true;

            aligned_snap.trackList{t} = trk;
        end
        aligned_R2{k} = aligned_snap;
    end
end
