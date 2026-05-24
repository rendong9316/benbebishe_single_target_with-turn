% =========================================================================
% run_track_fusion.m
% 逐帧航迹融合主循环
% =========================================================================
% 对已匹配的R1-R2航迹对, 在统一时间网格上逐帧融合
% 支持四种算法: SCC, BC, CI, FCI
% =========================================================================

function fused_snapshots = run_track_fusion(matched_pairs, trackSnapshots_R1, ...
        aligned_R2, params, method)

    n_frames = length(trackSnapshots_R1);
    n_pairs = length(matched_pairs);

    % 建立航迹ID到pair索引的映射
    r1_to_pair = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
    r2_to_pair = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
    for p = 1:n_pairs
        r1_to_pair(matched_pairs(p).R1_track_id) = p;
        r2_to_pair(matched_pairs(p).R2_track_id) = p;
    end

    % BC需要维护互协方差
    if strcmp(method, 'BC')
        P12_cell = cell(n_pairs, 1);
        for p = 1:n_pairs
            P12_cell{p} = zeros(4, 4);
        end
        has_prev = false(n_pairs, 1);
    end

    fused_snapshots = cell(n_frames, 1);

    for k = 1:n_frames
        snap_r1 = trackSnapshots_R1{k};
        snap_r2 = aligned_R2{k};

        fused_snap = struct('frameID', k, 'trackList', {{}});

        for p = 1:n_pairs
            r1_id = matched_pairs(p).R1_track_id;
            r2_id = matched_pairs(p).R2_track_id;

            % 查找本帧R1航迹
            trk1 = find_track(snap_r1, r1_id);
            % 查找本帧R2航迹
            trk2 = find_track(snap_r2, r2_id);

            % 跳过无有效UKF状态的航迹 (起始前/终止后)
            if ~isempty(trk1) && (isempty(trk1.ukf) || ~isfield(trk1.ukf,'x') || isempty(trk1.ukf.x))
                trk1 = [];
            end
            if ~isempty(trk2) && (isempty(trk2.ukf) || ~isfield(trk2.ukf,'x') || isempty(trk2.ukf.x))
                trk2 = [];
            end

            if isempty(trk1) && isempty(trk2)
                continue;  % 两源都无数据
            end

            fused_trk = struct();
            fused_trk.id = p;  % 融合航迹ID = pair index
            fused_trk.r1_id = r1_id;
            fused_trk.r2_id = r2_id;

            if ~isempty(trk1) && ~isempty(trk2)
                % 双源融合
                x1 = trk1.ukf.x;
                P1 = trk1.ukf.P;
                x2 = trk2.ukf.x;
                P2 = trk2.ukf.P;

                switch upper(method)
                    case 'SCC'
                        [x_f, P_f] = scc_fuse(x1, P1, x2, P2);
                        fused_trk.w = 0.5;  % SCC等效w=0.5

                    case 'CI'
                        [x_f, P_f, w_opt] = ci_fuse(x1, P1, x2, P2);
                        fused_trk.w = w_opt;

                    case 'FCI'
                        [x_f, P_f, w_fci] = fci_fuse(x1, P1, x2, P2);
                        fused_trk.w = w_fci;

                    case 'BC'
                        if has_prev(p)
                            % 预测互协方差 (仅1/2 Q贡献给互协方差)
                            dt = params.dt_sec;
                            F_cv_dt = [1, dt, 0, 0; 0, 1, 0, 0; 0, 0, 1, dt; 0, 0, 0, 1];
                            Q_half = trk1.ukf.Q * 0.5;
                            P12_pred = F_cv_dt * P12_cell{p} * F_cv_dt' + Q_half;

                            % 更新互协方差: 用迹收缩比近似 I-KH 的压缩效果
                            if isfield(trk1, 'P_pred') && isfield(trk2, 'P_pred') ...
                                    && ~isempty(trk1.P_pred) && ~isempty(trk2.P_pred) ...
                                    && trace(trk1.P_pred) > 1e-10 && trace(trk2.P_pred) > 1e-10
                                alpha1 = sqrt(max(1e-6, trace(trk1.ukf.P) / trace(trk1.P_pred)));
                                alpha2 = sqrt(max(1e-6, trace(trk2.ukf.P) / trace(trk2.P_pred)));
                                alpha = max(0.1, min(1.0, alpha1 * alpha2));
                                P12_new = alpha * P12_pred;
                            else
                                P12_new = 0.5 * P12_pred;
                            end

                            % 稳定性: 限制P12不超过 0.8*min(P1,P2)
                            max_p12 = 0.8 * min(diag(P1)) * eye(4);
                            for ii = 1:4
                                if P12_new(ii,ii) > max_p12(ii,ii)
                                    scale = sqrt(max_p12(ii,ii) / max(1e-10, P12_new(ii,ii)));
                                    P12_new(ii,:) = P12_new(ii,:) * scale;
                                    P12_new(:,ii) = P12_new(:,ii) * scale;
                                end
                            end
                        else
                            P12_new = zeros(4, 4);
                        end
                        [x_f, P_f] = bc_fuse(x1, P1, x2, P2, P12_new);
                        P12_cell{p} = P12_new;
                        has_prev(p) = true;
                        fused_trk.P12 = P12_new;

                    otherwise
                        error('Unknown fusion method: %s', method);
                end

                fused_trk.lat = x_f(3);
                fused_trk.lon = x_f(1);
                fused_trk.ukf_x = x_f;
                fused_trk.ukf_P = P_f;
                fused_trk.source = 'both';
                fused_trk.life = max(trk1.life, trk2.life);

            elseif ~isempty(trk1)
                % 仅R1有数据
                fused_trk.lat = trk1.ukf.x(3);
                fused_trk.lon = trk1.ukf.x(1);
                fused_trk.ukf_x = trk1.ukf.x;
                fused_trk.ukf_P = trk1.ukf.P;
                fused_trk.source = 'R1_only';
                fused_trk.life = trk1.life;
                if strcmp(method, 'BC')
                    has_prev(p) = false;
                end

            else  % 仅R2有数据
                fused_trk.lat = trk2.ukf.x(3);
                fused_trk.lon = trk2.ukf.x(1);
                fused_trk.ukf_x = trk2.ukf.x;
                fused_trk.ukf_P = trk2.ukf.P;
                fused_trk.source = 'R2_only';
                fused_trk.life = trk2.life;
                if strcmp(method, 'BC')
                    has_prev(p) = false;
                end
            end

            fused_snap.trackList{end+1} = fused_trk;
        end

        fused_snapshots{k} = fused_snap;
    end
end

function trk = find_track(snap, track_id)
    trk = [];
    if isempty(snap) || ~isfield(snap, 'trackList'), return; end
    for t = 1:length(snap.trackList)
        if snap.trackList{t}.id == track_id
            trk = snap.trackList{t};
            return;
        end
    end
end
