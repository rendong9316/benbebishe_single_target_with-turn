% =========================================================================
% ukf_filter_init.m
% =========================================================================
% 功能说明：
%   本函数执行UKF滤波器的单点初始化（首次量测初始化）。
%   当滤波器收到第一帧雷达量测数据（未初始化状态）时，无法直接进行
%   完整的预测-更新循环（因为还没有先验状态估计），所以需要先通过
%   第一帧量测来反算目标初始位置，并设置合理的不确定性（协方差）。
%
%   UKF算法中的角色：
%     这是UKF滤波循环的启动步骤。在没有先验信息的情况下，利用第一帧
%     量测建立滤波器的初始状态。初始化采用"零速假设"——假定目标初始
%     速度为零，并赋予较大的速度不确定度。
%
%   数学原理：
%     (1) 位置初始化：
%         通过球面反算函数 ukf_meas_to_latlon 将极坐标量测（斜距、方位角）
%         转换为WGS-84经纬度坐标，作为初始位置估计。
%         初始位置的不确定性由 ukf_P_pos_std 参数控制（默认值通常在
%         config/simulation_params.m 中设置，例如对应雷达距离和角度误差）。
%
%     (2) 速度初始化：
%         由于只有单帧量测，无法推算目标速度，因此速度分量初始化为0。
%         速度的不确定性由 ukf_P_vel_std 参数控制，通常设为一个远大于
%         预期速度的值（如10~20 m/s对应度/秒），以覆盖各种可能情况。
%
%     (3) 协方差矩阵：
%         P = diag([σ_pos^2, σ_vel^2, σ_pos^2, σ_vel^2])
%         采用对角矩阵形式，表示初始状态分量之间不相关。
%
%   输入参数：
%     ukf  : UKF滤波器结构体，必须包含：
%            - ukf.params.ukf_P_pos_std : 初始位置标准差（度）
%            - ukf.params.ukf_P_vel_std : 初始速度标准差（度/秒）
%            - 以及 ukf_meas_to_latlon 所需的雷达参数字段
%     meas : 量测结构体（struct），含以下字段：
%            - meas.range_meas   : 雷达测量的斜距（米）
%            - meas.azimuth_meas : 雷达测量的方位角（度）
%
%   输出参数：
%     ukf  : 初始化后的UKF结构体，包含：
%            - ukf.x           : 初始状态向量 [lon; 0; lat; 0]
%            - ukf.P           : 初始协方差矩阵（4×4对角阵）
%            - ukf.initialized : 设置为 true，标志滤波器已初始化
%
%   注意事项：
%     - 初始化只在第一帧执行一次，后续帧调用 ukf_filter_update 直接
%       进行预测-更新循环
%     - 零速初始化意味着滤波器需要几个更新周期才能收敛到正确的速度估计
%     - P_pos_std 一般取量测误差转换到经纬度域的标准差
%     - P_vel_std 建议取较大值以避免初始估计过于自信，导致收敛缓慢
%       （即"协方差低估"问题，滤波器会过度信任错误的初始速度估计）
% =========================================================================

function ukf = ukf_filter_init(ukf, meas, meas2)
    % ======== 步骤1：获取第一帧（及可选第二帧）雷达量测 ========
    rng_meas = meas.range_meas;
    az_meas = meas.azimuth_meas;

    % ======== 步骤2：极坐标→经纬度转换（球面反算） ========
    [lon, lat] = ukf_meas_to_latlon(ukf, rng_meas, az_meas);

    % ======== 步骤3：设置初始状态向量 ========
    lon_dot = 0.0;
    lat_dot = 0.0;
    if nargin >= 3 && ~isempty(meas2)
        % 两点初始化：利用前两帧量测估计初始速度
        [lon2, lat2] = ukf_meas_to_latlon(ukf, meas2.range_meas, meas2.azimuth_meas);
        dt_init = meas2.time_sec - meas.time_sec;
        if dt_init > 0.01
            lon_dot = (lon2 - lon) / dt_init;
            lat_dot = (lat2 - lat) / dt_init;
            % 速度合理性检验：位置噪声~30km时速度噪声可达1000+m/s，放宽至10-2000 m/s
            lat_mid = (lat + lat2) / 2;
            v_east = lon_dot * pi/180 * ukf.R_EARTH * cosd(lat_mid);
            v_north = lat_dot * pi/180 * ukf.R_EARTH;
            speed_ms = sqrt(v_east^2 + v_north^2);
            if speed_ms < 10 || speed_ms > 2000
                % 两点速度不合理（可能一点是杂波），退化为零速单点初始化
                % 使用当前帧点迹（meas2）作为初始位置
                lon = lon2;  lat = lat2;
                lon_dot = 0.0;  lat_dot = 0.0;
            end
        end
    end
    ukf.x = [lon; lon_dot; lat; lat_dot];

    % ======== 步骤4：构建初始协方差矩阵 ========
    pp = ukf.params.ukf_P_pos_std;
    pv = ukf.params.ukf_P_vel_std;
    ukf.P = diag([pp^2, pv^2, pp^2, pv^2]);

    % ======== 步骤5：标记滤波器已初始化 ========
    ukf.initialized = true;
end
