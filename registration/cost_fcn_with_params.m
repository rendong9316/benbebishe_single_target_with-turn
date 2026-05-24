% ============================================================================
% cost_fcn_with_params.m
% 空间配准 EML（Earth-Centered-Earth-Fixed 空间最大似然）优化的代价函数
% ============================================================================
%
% 【功能概述】
%   本函数是 estimate_biases.m 中 EML 精化阶段的目标函数（代价函数）。
%   给定一组候选的系统偏差值（雷达1和雷达2的距离偏置和方位偏置），
%   返回该组偏差对应的总代价值，即两部雷达校正后位置与标校点真实位置
%   在 ECEF 三维直角坐标系中的欧氏距离平方和。
%   代价越小，说明偏差估计越准确——两部雷达校正后越能同时指向标校点。
%
% 【在空间配准流程中的角色】
%   空间配准的整体流程为：
%      Step 1: estimate_biases.m —— 估计两部雷达的系统偏差（距离偏置、方位偏置）
%              ├── LS（直接最小二乘）：粗略估计，作为初值
%              └── EML（ECEF 空间优化）：以 LS 为初值，调用本函数（cost_fcn_with_params）
%                   作为 fmincon 的目标函数，精化偏差估计
%      Step 2: correct_measurements.m —— 用估计出的偏差校正所有量测
%      Step 3: align_radar_to_grid.m —— 将校正后的两部雷达航迹时间对齐到统一网格
%      Step 4: 后续融合处理
%   本函数服务于 Step 1 的 EML 阶段。
%
% 【数学原理 —— ECEF 空间代价函数】
%   设标校点 i 的真实经纬度为 (t_lon_i, t_lat_i)，其 ECEF 坐标为 te_i。
%   设雷达1的候选偏差为 (dr1, da1)，雷达2的候选偏差为 (dr2, da2)。
%
%   雷达1校正后的量测：
%     r1_corrected = r1_meas_i - dr1          （距离校正：减去系统距离偏置）
%     a1_corrected = a1_meas_i - da1          （方位角校正：减去系统方位偏置）
%     用球面 dead reckoning（目标点反算）得到雷达1推算的目标经纬度 (e1_lon, e1_lat)
%     将 (e1_lon, e1_lat) 转换到 ECEF 坐标系，得到 e1e
%
%   雷达2同理，得到 e2e。
%
%   代价函数定义为所有标校点的误差平方和：
%     total_cost = Σ_i [ ||e1e_i - te_i||^2 + ||e2e_i - te_i||^2 ]
%
%   其中 ||·|| 是欧氏距离（ECEF 空间中的三维距离，单位：米）。
%   除以 1e6 是为了数值稳定性（数值缩放，避免代价过大导致 fmincon 的梯度计算不稳定）。
%
%   物理含义：
%     如果偏差估计完全准确（dr1 = 真实距离偏置，da1 = 真实方位偏置），
%     那么雷达1校正后的 ECEF 位置应该恰好等于标校点 ECEF 位置，
%     即 ||e1e_i - te_i||^2 = 0。偏差不准时，该距离 > 0。
%     通过最小化所有标校点的总代价，fmincon 找到最优的偏差估计。
%
%   为什么在 ECEF 空间而非经纬度空间做优化？
%     - ECEF 是三维直角坐标系（X、Y、Z 是欧几里德空间），距离就是欧氏距离，
%       优化问题是标准的无约束/有约束最小二乘，梯度表达式简单，数值稳定。
%     - 如果直接在经纬度（球面坐标）上做最小二乘：
%       1. 经纬度的度量不统一（经度1度在不同纬度对应不同地面距离）
%       2. 纬度奇异性（接近极点时经度的物理意义退化）
%       3. 角度包裹问题（360° 和 0° 是同一方位，差值需要特殊处理）
%     - 因此先统一转到 ECEF 三维直角坐标系是标准做法。
%
% 【输入参数】
%   biases       - 4维向量 [dr1, da1, dr2, da2]，待优化的系统偏差候选值
%                   dr1: 雷达1距离偏置（米），正值表示雷达报告的距离比真实值大
%                   da1: 雷达1方位偏置（度），正值表示雷达报告的方位角比真实值大（偏右）
%                   dr2: 雷达2距离偏置（米）
%                   da2: 雷达2方位偏置（度）
%   cp_list      - 优化点集，cell 数组，每个 cell 元素是一个 struct，包含：
%                   truth: [lon, lat] 标校点真实经纬度
%                   r1_rng: 雷达1该帧的距离量测值（米，含偏置+噪声）
%                   r1_az:  雷达1该帧的方位角量测值（度，含偏置+噪声）
%                   r2_rng: 雷达2该帧的距离量测值（米）
%                   r2_az:  雷达2该帧的方位角量测值（度）
%   radar1_lon   - 雷达1部署位置的经度（度）
%   radar1_lat   - 雷达1部署位置的纬度（度）
%   radar2_lon   - 雷达2部署位置的经度（度）
%   radar2_lat   - 雷达2部署位置的纬度（度）
%
% 【返回值】
%   total        - 标量代价值（ECEF 空间误差平方和，已除以 1e6 做数值缩放）
%
% 【调用关系】
%   被 estimate_biases.m 中的匿名函数调用，作为 fmincon 的目标函数句柄
%   调用 coord_systems_lla_to_ecef（经纬度→ECEF转换）
%   调用 sphere_utils_destination_point（球面目标点反算）
%
% 【注意事项】
%   1. 本函数会被 fmincon 反复调用（每次迭代都要算一次代价），
%      因此内部逻辑尽量简洁、无冗余操作，确保优化收敛速度。
%   2. 代价进行了除以 1e6 的数值缩放，避免 ECEF 坐标的米级数值
%      （地球半径约 6.37e6 米）导致代价量级过大（~10^14），
%      从而影响 fmincon 基于梯度的收敛稳定性。
%   3. 目标点的高度（HAE）一律设为 0.0，因为我们在做水平空间配准，
%      不涉及高度偏差估计。
%
% ============================================================================

function total = cost_fcn_with_params(biases, cp_list, radar1_lon, radar1_lat, radar2_lon, radar2_lat, ...
        tx1_lon, tx1_lat, tx2_lon, tx2_lat)
    % 总代价函数：给定偏差候选值，返回 ECEF 空间中的总误差平方和

    %% ---- 从输入向量中解包四个偏差参数 ----
    % biases 是一个 [1x4] 或 [4x1] 的向量，按顺序存放：
    %   biases(1) = dr1：雷达1距离偏置（米）
    %   biases(2) = da1：雷达1方位偏置（度）
    %   biases(3) = dr2：雷达2距离偏置（米）
    %   biases(4) = da2：雷达2方位偏置（度）
    % fmincon 会把迭代过程中的候选解填入 biases 向量，本函数取出各分量进行评估
    dr1 = biases(1);   % 雷达1距离偏置（米）
    da1 = biases(2);   % 雷达1方位偏置（度）
    dr2 = biases(3);   % 雷达2距离偏置（米）
    da2 = biases(4);   % 雷达2方位偏置（度）

    %% ---- 初始化总代价 ----
    % total 将从 0.0 开始累加，最终是所有标校点的 ECEF 误差平方和
    total = 0.0;

    %% ---- 遍历所有优化点（标校点），逐个贡献代价 ----
    % cp_list 是一个 cell 数组，每个 cell 元素对应一个标校点/一帧量测数据
    % length(cp_list) 返回 cell 数组中的元素个数
    for j = 1:length(cp_list)

        %% -- 取出第 j 个优化点的信息 --
        % cp_list{j} 用花括号 {} 读取 cell 数组的第 j 个元素，取出的是 struct
        cp = cp_list{j};

        %% -- 取出标校点的真实经纬度坐标 --
        % cp.truth 是一个 [lon, lat] 的 1x2 向量
        % cp.truth(1) 是经度（度），cp.truth(2) 是纬度（度）
        t_lon = cp.truth(1);   % 标校点真实经度（度）
        t_lat = cp.truth(2);   % 标校点真实纬度（度）

        %% -- 将标校点真实经纬度转换到 ECEF 直角坐标系 --
        % coord_systems_lla_to_ecef(lat, lon, alt)
        %   输入：纬度（度）、经度（度）、椭球高度（米，此处设为 0.0 表示海平面）
        %   输出：te 是 1x3 向量 [Xe, Ye, Ze]，单位：米
        %   坐标系：ECEF（Earth-Centered, Earth-Fixed），原点在地球质心
        %   X 轴指向本初子午线与赤道交点，Z 轴指向北极，Y 轴构成右手系
        te = coord_systems_lla_to_ecef(t_lat, t_lon, 0.0);

        %% ================================================================
        %% 雷达1 部分：校正量测 → 推算目标位置 → 转 ECEF → 计算误差
        %% ================================================================

        %% -- 距离校正：量测距离减去候选距离偏置 --
        % 如果 dr1 恰好等于真实的系统距离偏置，那么 r1c 就是无偏的距离估计
        r1c = cp.r1_rng - dr1;   % 雷达1校正后距离（米）

        %% -- 方位角校正：量测方位角减去候选方位偏置 --
        a1c = cp.r1_az - da1;   % 雷达1校正后方位角（度）

        %% -- 双基地反解：从群距离和方位角求 r1（目标到Rx的地表距离） --
        baseline1 = sphere_utils_haversine_distance(tx1_lon, tx1_lat, radar1_lon, radar1_lat);
        tx_az1 = sphere_utils_azimuth(radar1_lon, radar1_lat, tx1_lon, tx1_lat);
        phi1 = a1c - tx_az1;
        r1_dist = 0.5 * (r1c^2 - baseline1^2) / (r1c - baseline1 * cosd(phi1));
        [e1_lon, e1_lat] = sphere_utils_destination_point(radar1_lon, radar1_lat, r1_dist, a1c);

        %% -- 将雷达1推算的目标经纬度转为 ECEF 坐标 --
        e1e = coord_systems_lla_to_ecef(e1_lat, e1_lon, 0.0);

        %% -- 计算雷达1校正后位置与标校点真实位置的 ECEF 误差平方 --
        % (e1e - te) 是 1x3 向量，表示 ECEF 空间中三个坐标分量上的偏差
        % (e1e - te).^2 是逐元素平方，得到 [dx^2, dy^2, dz^2]
        % sum(...) 将三个分量求和，得到 ||e1e - te||^2（ECEF 欧氏距离平方，单位：m^2）
        % 累加到 total
        total = total + sum((e1e - te).^2);

        %% ================================================================
        %% 雷达2 部分：与雷达1完全相同的处理流程
        %% ================================================================

        %% -- 距离校正 --
        r2c = cp.r2_rng - dr2;   % 雷达2校正后距离（米）

        %% -- 方位角校正 --
        a2c = cp.r2_az - da2;   % 雷达2校正后方位角（度）

        %% -- 双基地反解：从群距离和方位角求 r1（目标到Rx的地表距离） --
        baseline2 = sphere_utils_haversine_distance(tx2_lon, tx2_lat, radar2_lon, radar2_lat);
        tx_az2 = sphere_utils_azimuth(radar2_lon, radar2_lat, tx2_lon, tx2_lat);
        phi2 = a2c - tx_az2;
        r2_dist = 0.5 * (r2c^2 - baseline2^2) / (r2c - baseline2 * cosd(phi2));
        [e2_lon, e2_lat] = sphere_utils_destination_point(radar2_lon, radar2_lat, r2_dist, a2c);

        %% -- 转为 ECEF 坐标 --
        e2e = coord_systems_lla_to_ecef(e2_lat, e2_lon, 0.0);

        %% -- 累加雷达2的误差平方 --
        total = total + sum((e2e - te).^2);

        %% ---- 数值缩放：总代价除以 1e6 以保证数值稳定性 ----
        % ECEF 坐标在米量级（~6.37e6 米），误差平方约在 10^6 ~ 10^10 量级
        % 如果不做缩放，fmincon 的梯度计算和 Hessian 逼近可能因数值量级过大而不稳定
        % 除以 1e6 将代价值降到合理的数值范围（~1 ~ 10^4），提高优化收敛的数值稳定性
        % 注意：缩放不改变最优解的位置（单调变换），只影响 fmincon 内部的数值计算
        total = total / 1e6;

    end  % for 循环结束——所有标校点处理完毕

end  % 函数 cost_fcn_with_params 结束
