% =========================================================================
% det2nanyang_point.m — 当前项目检测格式 → 南阳点迹格式转换器
%
% 输入:
%   det_list  — 当前项目的检测结构体数组（单帧）
%              字段: range_meas(m), azimuth_meas(deg), radial_v_meas(m/s),
%                    lon(deg), lat(deg), SNR(dB), is_clutter, frameID, detID
%   frame_id  — 当前帧号
%   time_stamp — MATLAB datenum 时间戳
%
% 输出:
%   point_list — 南阳格式的点迹结构体数组
%               字段: frameID, time, prange(km), pvr(m/s), paz(deg),
%                     drange(km), dvr(m/s), daz(deg), pd_range, pd_az,
%                     lat, lon, Rbin, Dbin, Abin, snr, amp,
%                     beampattern, channvalue, ambgNum, ionoMode
% =========================================================================
function point_list = det2nanyang_point(det_list, frame_id, time_stamp)
    if isempty(det_list)
        point_list = [];
        return;
    end

    n_dets = length(det_list);
    % 预分配结构体数组
    point_list = struct(...
        'frameID',     cell(1, n_dets), ...
        'time',        cell(1, n_dets), ...
        'ionoMode',    cell(1, n_dets), ...
        'prange',      cell(1, n_dets), ...
        'paz',         cell(1, n_dets), ...
        'pvr',         cell(1, n_dets), ...
        'drange',      cell(1, n_dets), ...
        'daz',         cell(1, n_dets), ...
        'dvr',         cell(1, n_dets), ...
        'pd_range',    cell(1, n_dets), ...
        'pd_az',       cell(1, n_dets), ...
        'lat',         cell(1, n_dets), ...
        'lon',         cell(1, n_dets), ...
        'Rbin',        cell(1, n_dets), ...
        'Dbin',        cell(1, n_dets), ...
        'Abin',        cell(1, n_dets), ...
        'snr',         cell(1, n_dets), ...
        'amp',         cell(1, n_dets), ...
        'beampattern', cell(1, n_dets), ...
        'channvalue',  cell(1, n_dets), ...
        'ambgNum',     cell(1, n_dets)  ...
    );

    for d = 1:n_dets
        dp = det_list(d);

        % 基本标识
        point_list(d).frameID  = frame_id;
        point_list(d).time     = time_stamp;
        point_list(d).ionoMode = 5;  % 默认电离层模式

        % 物理量测（南阳用 km 为单位存储距离！）
        if isfield(dp, 'range_meas')
            point_list(d).prange = dp.range_meas / 1000.0;  % m → km
        else
            point_list(d).prange = 0;
        end
        if isfield(dp, 'radial_vel_meas') && ~isnan(dp.radial_vel_meas)
            point_list(d).pvr = dp.radial_vel_meas;  % m/s
        elseif isfield(dp, 'radial_v_meas') && ~isnan(dp.radial_v_meas)
            point_list(d).pvr = dp.radial_v_meas;    % m/s (备用字段名)
        else
            point_list(d).pvr = 0;
        end
        if isfield(dp, 'azimuth_meas')
            point_list(d).paz = dp.azimuth_meas;  % deg
        else
            point_list(d).paz = 0;
        end

        % 检测量测（仿真无 pd 系数，与物理量测相同）
        point_list(d).drange = point_list(d).prange;  % km
        point_list(d).dvr    = point_list(d).pvr;     % m/s
        point_list(d).daz    = point_list(d).paz;     % deg

        % pd 系数（仿真 = 1.0 / 0.0）
        point_list(d).pd_range = 1.0;
        point_list(d).pd_az    = 0.0;

        % 地理位置
        if isfield(dp, 'lat') && ~isnan(dp.lat)
            point_list(d).lat = dp.lat;
            point_list(d).lon = dp.lon;
        else
            point_list(d).lat = NaN;
            point_list(d).lon = NaN;
        end

        % 距离/多普勒/方位 bin — 必须唯一！
        % fun_remove_assc_pts_from_pointlist 根据 (Rbin,Dbin,Abin) 三元组
        % 来识别模糊点迹并全部删除。仿真无模糊，但需保证每个点迹的
        % 三元组唯一，否则删除一个关联点会误删同帧其他点迹。
        point_list(d).Rbin = d;          % 帧内唯一索引
        point_list(d).Dbin = d;          % 帧内唯一索引
        point_list(d).Abin = frame_id;   % 帧号，区分不同帧

        % 检测质量信息
        if isfield(dp, 'SNR')
            point_list(d).snr = dp.SNR;
        else
            point_list(d).snr = 10;
        end
        point_list(d).amp         = 1.0;
        point_list(d).beampattern = 1.0;
        point_list(d).channvalue  = 1.0;
        point_list(d).ambgNum     = 0;  % 无速度模糊
    end
end
