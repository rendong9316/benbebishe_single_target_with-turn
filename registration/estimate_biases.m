% ============================================================================
% estimate_biases.m
% 空间配准模块 —— 系统偏差估计（直接最小二乘 LS + EML 精化双阶段算法）
% ============================================================================
%
% 【功能概述】
%   本函数是空间配准（Spatial Registration）流程中的核心模块，负责从多帧
%   标校点量测数据中估计两部雷达的系统偏差（距离偏置和方位角偏置）。
%   算法分为两个阶段：
%     Stage 1 (LS): 直接最小二乘 —— 逐帧计算"量测值 - 真实值"，取统计平均
%                   作为偏差的粗略估计（提供 fmincon 的初值）。
%     Stage 2 (EML): ECEF 空间最大似然精化 —— 以 LS 结果为初值，在 ECEF
%                    三维直角坐标系中通过 fmincon 数值优化，精调偏差估计。
%
% 【数学原理 —— 直接最小二乘 (LS)】
%   雷达量测模型：
%     r_meas = r_true + dr_sys + n_r        （距离量测 = 真实距离 + 系统偏置 + 噪声）
%     a_meas = a_true + da_sys + n_a        （方位量测 = 真实方位 + 系统偏置 + 噪声）
%   其中：
%     r_meas, a_meas := 雷达报告的距离和方位角（含偏置+噪声）
%     r_true, a_true := 目标在雷达坐标系中的真实距离和方位角
%     dr_sys, da_sys := 恒定的系统距离偏置和方位偏置（我们要估计的量）
%     n_r, n_a := 零均值高斯噪声（服从 N(0, sigma^2)）
%
%   对于标校点 i（已知真实经纬度，可以算出真实的球面距离/方位角）：
%     dr_i = r_meas_i - r_true_i = dr_sys + n_r_i
%     da_i = a_meas_i - a_true_i = da_sys + n_a_i
%
%   由于 n_r_i 是零均值高斯噪声，对 N 个标校点取平均：
%     dr_ls = (1/N) * sum(dr_i) = dr_sys + (1/N) * sum(n_r_i) → dr_sys  (当 N 足够大)
%   同理 da_ls → da_sys。
%   因此 LS 估计（算术平均）在高斯噪声下是系统偏差的最优无偏估计（MLE）。
%
% 【数学原理 —— EML 精化 (ECEF 空间优化)】
%   LS 估计的问题：
%   1. 它仅利用"量测-真值"的标量差，没有利用两部雷达同时观测同一目标的空间
%      几何约束——两部雷达校正后都应该同时指向同一个标校点。
%   2. 方位角差值需要做角度包裹（wrap-around）处理，在边界附近可能引人误差。
%
%   EML 的改进：
%   将偏差估计问题转化为 ECEF 空间中的最小化问题：
%     min Σ_i [ ||ECEF(radar1_corrected_i) - ECEF(truth_i)||^2
%             + ||ECEF(radar2_corrected_i) - ECEF(truth_i)||^2 ]
%   其中：
%     radar1_corrected_i = 从雷达1位置出发，(r1_meas_i - dr1, a1_meas_i - da1)
%                          反算出的经纬度 → 转 ECEF
%     truth_i 的 ECEF = 从标校点真实经纬度转换的 ECEF 坐标
%
%   物理含义：如果偏差估计绝对准确，两部雷达校正后的位置应该精确落在标校点上，
%   即所有标校点的 ECEF 距离误差为 0。代价函数越小，偏差估计越接近真值。
%
%   为什么选 ECEF 而非 ENU/经纬度？
%   1. ECEF 是真正的三维欧氏空间，梯度在全局均匀
%   2. 经纬度在极区有奇异性（经度的物理意义退化）
%   3. ENU（切平面）只在局部有效，多标校点分布较广时切平面畸变不可忽略
%   4. ECEF 是全局笛卡尔系，标准的最小二乘理论完全适用
%
%   fmincon 求解器：
%     MATLAB 的 fmincon (find minimum of constrained nonlinear multivariable function)
%     使用 SQP (Sequential Quadratic Programming) 算法：
%       1. 在当前点 x_k 处将原始非线性优化近似为二次规划（QP）子问题
%       2. 求解 QP 子问题得到搜索方向 d_k
%       3. 沿 d_k 做线搜索确定步长 alpha_k
%       4. 更新 x_{k+1} = x_k + alpha_k * d_k
%       5. 重复直到收敛（梯度范数 < OptimalityTolerance）
%     fmincon 内部通过有限差分逼近梯度和 Hessian，不需要手动提供导数。
%
% 【在空间配准流程中的角色】
%   本函数是整个空间配准的第一步（偏差估计），其输出（est）将作为
%   correct_measurements.m 的输入参数，用于校正所有雷达量测数据。
%
% 【输入参数】
%   r1_meas       - 雷达1的量测序列，cell array of struct
%                   每个 cell 元素对应一帧量测（struct），包含字段：
%                     range_meas:     距离量测（米）
%                     azimuth_meas:   方位角量测（度）
%                     radial_vel_meas:径向速度量测（m/s，本函数中未使用）
%                   漏检帧（无目标检测）则为空数组 []
%   r2_meas       - 雷达2的量测序列，格式同 r1_meas
%   truth_points  - 标校点真实位置，[N x 2] 矩阵
%                   第 i 行 = [lon_i, lat_i]（经度,纬度，单位：度）
%   radar1_lon    - 雷达1部署位置的经度（度）
%   radar1_lat    - 雷达1部署位置的纬度（度）
%   radar2_lon    - 雷达2部署位置的经度（度）
%   radar2_lat    - 雷达2部署位置的纬度（度）
%
% 【返回值】
%   est           - 结构体 struct，包含估计出的四个系统偏置参数：
%                     dr1: 雷达1距离偏置估计（米）
%                     da1: 雷达1方位偏置估计（度）
%                     dr2: 雷达2距离偏置估计（米）
%                     da2: 雷达2方位偏置估计（度）
%
% 【调用关系】
%   调用：cost_fcn_with_params（EML 代价函数）
%         sphere_utils_haversine_distance（球面 Haversine 距离计算）
%         sphere_utils_azimuth（球面方位角计算）
%   被调用：main.m 或其他主控脚本，作为空间配准的第一步
%
% 【注意事项】
%   1. 至少需要3个有效标校点才能做有意义的统计估计（n_cal < 3 时直接返回0偏置）
%   2. 方位角差值需要做 [-180, 180] 范围的角度归化处理
%   3. EML 优化点的下采样（最多50个）是为了控制 fmincon 的计算复杂度
%   4. 优化边界目前设定较大（距离 +/- 50000m，方位 +/- 10°），以适应大偏差场景
%   5. 代码中第 274 行 `x_opt = x0` 说明作者最终认定 LS 结果优于 fmincon 精化结果
%
% ============================================================================

function est = estimate_biases(r1_meas, r2_meas, truth_points, ...
                                radar1_lon, radar1_lat, radar2_lon, radar2_lat, ...
                                tx1_lon, tx1_lat, tx2_lon, tx2_lat)
    % 估计系统偏置（直接LS + EML精化）
    %
    % 输入:
    %   r1_meas: 雷达1含偏量测，cell array of struct，每个元素是一帧量测
    %            每帧量测是 struct，含字段：range_meas, azimuth_meas, radial_vel_meas
    %   r2_meas: 雷达2含偏量测，同上
    %   truth_points: 标校点真实经纬度，[N x 2] 矩阵，每行 [lon, lat]
    %   radar1_lon, radar1_lat: 雷达1经纬度（度）
    %   radar2_lon, radar2_lat: 雷达2经纬度（度）
    %
    % 返回:
    %   est: 结构体，四个估计出的偏差值
    %        dr1: 雷达1距离偏置（米），正=多报
    %        da1: 雷达1方位偏置（度），正=偏右
    %        dr2: 雷达2距离偏置（米）
    %        da2: 雷达2方位偏置（度）

    %% ============================================================================
    %% 第一部分：数据预处理 —— 确定有多少个标校点可用
    %% ============================================================================

    % 两部雷达的量测帧数可能不同（两部雷达是异步采样的，采样时刻不同）
    % min(length(r1_meas), length(r2_meas)) 取较小者，保证每帧都有两部雷达的数据
    % length(X) 对于 cell 数组返回 cell 中元素的数量，即帧数
    total_pts = min(length(r1_meas), length(r2_meas));

    % 标校点的真实位置数组大小
    % size(truth_points, 1) 返回 truth_points 矩阵的行数，即标校点总数
    n_pts = size(truth_points, 1);

    % 实际用于标校的点数：取量测帧数和标校点数中较小的那个
    % 如果标校点比帧数多（n_pts > total_pts），只能用前 total_pts 个标校点
    % 如果帧数比标校点多（total_pts > n_pts），只能用 n_pts 个标校点
    n_cal = min(total_pts, n_pts);

    %% ============================================================================
    %% 安全性检查 —— 标校点不足3个则无法估计，直接返回0偏置
    %% ============================================================================
    % 至少需要3个点才能做有意义的统计平均
    % 因为统计估计需要足够的样本量来抑制噪声（大数定律）
    % 同时LS估计的方差在 N>=3 时才有合理的置信度
    if n_cal < 3
        % 创建结构体，将四个偏差估计都设为 0
        % struct('field1', val1, 'field2', val2, ...) 创建带命名字段的结构体
        est = struct('dr1', 0, 'da1', 0, 'dr2', 0, 'da2', 0);
        return;   % 提前返回，结束函数执行
    end

    % 确定要使用的标校点索引序列：1, 2, 3, ..., n_cal
    % 1:n_cal 在 MATLAB 中生成行向量 [1, 2, 3, ..., n_cal]
    cal_idxs = 1:n_cal;

    %% ============================================================================
    %% Step 1: 直接最小二乘估计（LS —— Least Squares）
    %% ============================================================================
    % 核心思想：偏差 = 量测量 - 真实值
    % 由于每帧量测都带有随机噪声，利用噪声的零均值特性，
    % 对多帧取均值可以抑制噪声影响，即：
    %   偏差估计 ≈ mean_i(量测_i - 真实值_i) = 真实偏差 + mean_i(噪声_i)
    %   当帧数 N 足够大时，mean_i(噪声_i) → 0（大数定律）
    %
    % 这在高斯噪声假设下是最优无偏估计（MLE，Maximum Likelihood Estimator）：
    %   假设噪声服从 N(0, sigma^2)，则样本均值是期望的 MLE
    %
    % 为什么 LS 也叫"直接最小二乘"？
    %   设 dr 为需要估计的距离偏置，对每个标校点 i 有：
    %     dr_i = r_meas_i - r_true_i = dr_true + n_i
    %   最小化 Σ (dr_i - dr)^2 得到 dr_ls = mean(dr_i)
    %   所以取平均就是最小二乘的解

    % 预分配空数组，用于收集每一帧的偏差值
    % [] 表示空矩阵（0x0），在 MATLAB 中既是空也是行列向量的起点
    % end+1 索引在每次赋值时自动在数组末尾追加新元素（动态扩展）
    dr1_list = [];   % 雷达1距离偏差列表（米），每帧追加一个元素
    da1_list = [];   % 雷达1方位偏差列表（度），每帧追加一个元素
    dr2_list = [];   % 雷达2距离偏差列表（米）
    da2_list = [];   % 雷达2方位偏差列表（度）

    %% --------------------------------------------------------------------------
    %% 逐帧计算：量测值 - 真实值 = 偏差（含噪声的观测偏差）
    %% --------------------------------------------------------------------------
    % for i = 1:n_cal 遍历每一个可用标校点（帧）
    for i = 1:n_cal

        %% 取出第 i 帧的量测数据
        % cell 数组用花括号 { } 读取元素，圆括号 ( ) 返回的是 cell 而不是内容
        % r1_meas{cal_idxs(i)} 返回该 cell 位置存储的 struct，内容如：
        %   struct('range_meas', 50000, 'azimuth_meas', 45.2, 'radial_vel_meas', 0)
        m1 = r1_meas{cal_idxs(i)};   % 雷达1第 i 帧量测 struct
        m2 = r2_meas{cal_idxs(i)};   % 雷达2第 i 帧量测 struct

        %% 跳过漏检帧
        % isempty(m1) 检测 m1 是否为空数组（雷达未检测到目标）
        % 如果任一部雷达漏检，则该帧的偏差无法计算，跳过
        if isempty(m1) || isempty(m2), continue; end

        %% 取出第 i 个标校点的真实经纬度坐标
        % truth_points 的行数可能大于 n_cal，此处用 i 而非 cal_idxs(i)
        % 因为标校点索引天然与帧索引对齐（truth_points(1,:) 对应第一帧）
        t_lon = truth_points(i, 1);   % 第 i 行第 1 列 = 经度（度）
        t_lat = truth_points(i, 2);   % 第 i 行第 2 列 = 纬度（度）

        %% -----------------------------------------------------------------------
        %% 雷达1：计算该帧的距离偏差和方位偏差
        %% -----------------------------------------------------------------------

        %% 计算雷达1到标校点的真实球面距离
        % sphere_utils_haversine_distance(lon1, lat1, lon2, lat2)
        %   使用 Haversine 公式计算球面上两点间的大圆距离
        %   输入：经纬度（度），输出：距离（米）
        %   Haversine 公式：a = sin²(Δlat/2) + cos(lat1)*cos(lat2)*sin²(Δlon/2)
        %                    c = 2 * atan2(√a, √(1-a))
        %                    d = R * c，其中 R = 6371000 米（地球平均半径）
        %   相比余弦公式，Haversine 在极近点时数值更稳定（小角度差）
        %% 计算双基地真实群距离：Tx→目标 + 目标→Rx
        true_rng1 = sphere_utils_haversine_distance(tx1_lon, tx1_lat, t_lon, t_lat) ...
                  + sphere_utils_haversine_distance(radar1_lon, radar1_lat, t_lon, t_lat);

        %% 计算雷达1到标校点的真实球面方位角
        % sphere_utils_azimuth(lon1, lat1, lon2, lat2)
        %   返回从点1看向点2的方位角（北偏东为正，顺时针方向）
        %   返回值范围：[0, 360) 度
        %   数学公式：θ = atan2( sin(Δlon)*cos(lat2),
        %                         cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(Δlon) )
        true_az1 = sphere_utils_azimuth(radar1_lon, radar1_lat, t_lon, t_lat);

        %% 计算距离偏差：量测值 - 真实值
        % dr1_list(end+1) = value 在 dr1_list 数组末尾追加一个元素
        % 如果 dr1_list 原来是 [100, 200, 150]，追加后变为 [100, 200, 150, 新值]
        % m1.range_meas 是结构体字段访问（点号访问），获取距离量测值（米）
        dr1_list(end+1) = m1.range_meas - true_rng1;

        %% 计算方位角偏差：量测值 - 真实值，并处理角度包裹
        % 方位角是角度量，有循环性（360° = 0°）
        % 例如：量测 = 350°，真实 = 10°，实际偏差应该是 -20°（不是 340°）
        % 归化到 [-180°, 180°] 范围
        daz1 = m1.azimuth_meas - true_az1;   % 直接相减得到原始角度差

        % 处理角度包裹：将差值归化到 [-180, 180] 范围
        if daz1 > 180         % 如果差值 > 180°，说明实际偏差是负向的（绕了一圈）
            daz1 = daz1 - 360;   % 例如：350° - 10° = 340° → 340° - 360° = -20° ✓
        elseif daz1 < -180    % 如果差值 < -180°，同理做正向归化
            daz1 = daz1 + 360;   % 例如：-190° → -190° + 360° = 170°（等效为正向差）
        end
        % 现在 daz1 在 [-180, 180] 范围内，正值为顺时针偏差（偏右），负值为逆时针偏差（偏左）
        da1_list(end+1) = daz1;

        %% -----------------------------------------------------------------------
        %% 雷达2：与雷达1完全相同的处理流程
        %% -----------------------------------------------------------------------

        %% 计算雷达2到标校点的真实球面距离
        %% 计算双基地真实群距离：Tx→目标 + 目标→Rx
        true_rng2 = sphere_utils_haversine_distance(tx2_lon, tx2_lat, t_lon, t_lat) ...
                  + sphere_utils_haversine_distance(radar2_lon, radar2_lat, t_lon, t_lat);

        %% 计算雷达2到标校点的真实球面方位角
        true_az2 = sphere_utils_azimuth(radar2_lon, radar2_lat, t_lon, t_lat);

        %% 距离偏差 = 量测 - 真实
        dr2_list(end+1) = m2.range_meas - true_rng2;

        %% 方位角偏差，同样做 [-180, 180] 的角度归化
        daz2 = m2.azimuth_meas - true_az2;   % 原始角度差
        if daz2 > 180                         % 正向超出，减一圈
            daz2 = daz2 - 360;
        elseif daz2 < -180                    % 负向超出，加一圈
            daz2 = daz2 + 360;
        end
        da2_list(end+1) = daz2;

    end  % for 循环结束——所有标校点的偏差记录已收集完毕

    %% ============================================================================
    %% 再次检查 —— 收集到的有效偏差数据点不足3个则直接返回0偏置
    %% ============================================================================
    % 虽然前面 n_cal >= 3，但可能有漏检帧被跳过了（continue）
    % 导致实际有效的偏差记录数量可能不足3个，需要再次检查
    % length(dr1_list) 返回 dr1_list 数组中实际元素的数量
    if length(dr1_list) < 3
        est = struct('dr1', 0, 'da1', 0, 'dr2', 0, 'da2', 0);
        return;
    end

    %% ============================================================================
    %% LS 估计：对所有有效帧的偏差取统计均值
    %% ============================================================================
    % mean(X) 计算向量 X 的算术平均值：sum(X) / length(X)
    % 这就是"直接最小二乘"的解——求算术平均
    % 在高斯噪声下，样本均值是总体期望的 MLE（最大似然估计）
    % 也是 BLUE（最佳线性无偏估计）
    dr1_ls = mean(dr1_list);   % 雷达1距离偏差的 LS 估计（米）
    da1_ls = mean(da1_list);   % 雷达1方位偏差的 LS 估计（度）
    dr2_ls = mean(dr2_list);   % 雷达2距离偏差的 LS 估计（米）
    da2_ls = mean(da2_list);   % 雷达2方位偏差的 LS 估计（度）

    %% ============================================================================
    %% Step 2: EML 精化 —— 在 ECEF 空间中通过 fmincon 数值优化精调偏差
    %% ============================================================================
    % LS 估计的局限性：
    %   1. 仅利用了每个雷达独立的"量测-真值"标量差，没有利用两部雷达同时
    %      看到同一目标这一空间几何约束
    %   2. 没有在统一的坐标系（ECEF）中做全局一致性校核
    %
    % EML（ECEF Maximum Likelihood）思想：
    %   两部雷达校正后的 ECEF 位置应收敛到同一个点（标校点的真实 ECEF 位置）
    %   建立代价函数：总误差 = Σ (||ECEF_雷达1_校正 - ECEF_真实||²
    %                              + ||ECEF_雷达2_校正 - ECEF_真实||²)
    %   用数值优化算法（fmincon）最小化该代价，搜索最优偏差
    %
    % 为什么在 ECEF 空间而非经纬度/球面空间？
    %   - ECEF 是三维直角坐标系（X, Y, Z 在米量级），欧氏距离有明确的物理意义
    %   - 避免了球面坐标的循环性（角度包裹）和非线性奇异性（极点附近）
    %   - fmincon 的梯度计算在直角坐标系中更稳定

    %% --------------------------------------------------------------------------
    %% 下采样：为减少优化计算量，最多取50个标校点参与 EML 优化
    %% --------------------------------------------------------------------------
    % fmincon 每次迭代都需要评估代价函数 cost_fcn_with_params
    % 代价函数中要做球面目标点反算和 ECEF 坐标变换，计算量较大
    % 以 100 个标校点为例，每次代价评估需要 200 次（两部雷达×100点）
    %   coord_systems_lla_to_ecef 调用和 200 次 sphere_utils_destination_point 调用
    % 为节省计算时间，均匀抽取不超过 50 个点
    % 50 是经验值：在精度和速度之间取得平衡

    % opt_n：优化点数量，不超过 50
    % min(50, length(cal_idxs)) 和 50 以及现有索引数取最小值
    opt_n = min(50, length(cal_idxs));

    % opt_step：采样步长
    % floor(X) 向下取整，max(1, ...) 保证步长至少为 1（至少取1个点）
    % 例如：length(cal_idxs) = 120，opt_n = 50
    %       floor(120/50) = 2 → 每隔2个取1个 → 取到约 60 个 → 后面会截断到50
    opt_step = max(1, floor(length(cal_idxs) / opt_n));

    % 均匀采样：从索引 1 开始，步长为 opt_step，到 cal_idxs 末尾
    % cal_idxs(1:opt_step:end) 是 MATLAB 的切片语法：
    %   起始:步长:结束，等价于 Python 的 cal_idxs[0::opt_step]
    opt_idxs = cal_idxs(1:opt_step:end);

    % 如果采样后仍然超过 opt_n 个点，截取前 opt_n 个
    % min(opt_n, length(opt_idxs)) 确保截断数不超过数组实际长度
    opt_idxs = opt_idxs(1:min(opt_n, length(opt_idxs)));

    %% --------------------------------------------------------------------------
    %% 构建优化点集 cp_list —— 每个元素包含一个标校点的完整信息
    %% --------------------------------------------------------------------------
    % cp_list 是 cell 数组，会被传给 cost_fcn_with_params 作为代价函数的输入
    % 每个 cell 元素是一个 struct，包含该标校点的：
    %   truth:   真实经纬度 [lon, lat]
    %   r1_rng:  雷达1距离量测
    %   r1_az:   雷达1方位量测
    %   r2_rng:  雷达2距离量测
    %   r2_az:   雷达2方位量测
    %
    % {} 表示预分配一个空 cell 数组（0x0 cell）
    cp_list = {};

    % 遍历每个优化点的索引
    for i = 1:length(opt_idxs)

        % 取出当前优化点对应的原始帧索引
        idx = opt_idxs(i);

        % 取出该帧的量测数据
        m1 = r1_meas{idx};   % 雷达1量测 struct
        m2 = r2_meas{idx};   % 雷达2量测 struct

        % 如果该帧漏检，跳过（跳过此点，不加入优化点集）
        if isempty(m1) || isempty(m2), continue; end

        %% 构建优化点结构体，并追加到 cp_list 末尾
        % struct('field1', val1, 'field2', val2, ...) 创建结构体，字段用单引号括起来
        % cp_list{end+1} = ... 在 cell 数组末尾追加新元素
        % truth_points(idx, :) 取出第 idx 行的所有列（即 [lon, lat]）
        cp_list{end+1} = struct( ...
            'truth', truth_points(idx, :), ...   % 标校点真实经纬度 [lon, lat]
            'r1_rng', m1.range_meas, ...         % 雷达1该帧距离量测（米）
            'r1_az', m1.azimuth_meas, ...        % 雷达1该帧方位角量测（度）
            'r2_rng', m2.range_meas, ...         % 雷达2该帧距离量测（米）
            'r2_az', m2.azimuth_meas);           % 雷达2该帧方位角量测（度）

    end  % for 循环结束——所有优化点信息已收集完毕

    %% --------------------------------------------------------------------------
    %% 定义代价函数句柄 —— 将除 biases 外的参数"绑定"到匿名函数中
    %% --------------------------------------------------------------------------
    % fmincon 要求目标函数的形式为 f(x)，只接受优化变量 x 这一个输入
    % 但 cost_fcn_with_params 需要 6 个参数：(biases, cp_list, ..., radar2_lat)
    %
    % 解决方案：用匿名函数（anonymous function）将多余参数"预绑定"
    % @(biases) cost_fcn_with_params(biases, cp_list, ...) 语法：
    %   - @ 符号创建函数句柄（function handle）
    %   - (biases) 是匿名函数的形参列表，这里只有一个形参
    %   - 函数体调用 cost_fcn_with_params，并将 biases 作为第一个实参传入
    %   - cp_list, radar1_lon, ... 这些变量在匿名函数创建时被"捕获"（闭包）
    %
    % 这样 fmincon 调用 cost_fcn(x) 时，x 会传入 biases，其余参数由闭包提供
    cost_fcn = @(biases) cost_fcn_with_params(biases, cp_list, radar1_lon, radar1_lat, radar2_lon, radar2_lat, ...
        tx1_lon, tx1_lat, tx2_lon, tx2_lat);

    %% --------------------------------------------------------------------------
    %% 优化配置与执行 —— 调用 fmincon 进行有约束非线性优化
    %% --------------------------------------------------------------------------

    %% 优化初值 x0：以 LS 估计结果作为 fmincon 的起点
    % 为什么要用 LS 结果作为初值？
    % 1. LS 结果已经在真实解附近（高斯噪声下的无偏估计）
    % 2. 避免 fmincon 从远处搜索（可能陷入局部最优）
    % 3. 减少迭代次数，加快收敛
    x0 = [dr1_ls, da1_ls, dr2_ls, da2_ls];

    % 将初值输出到命令行，供调试查看
    % disp(X) 显示变量 X 的值
    disp(x0);

    %% --------------------------------------------------------------------------
    %% 优化边界设定 —— fmincon 的变量上下界约束
    %% --------------------------------------------------------------------------
    % 当前仿真配置的系统偏差假设量级：
    %   雷达1: dr1 ≈ 20000 m, da1 ≈ -3 deg
    %   雷达2: dr2 ≈ -15000 m, da2 ≈ 3.5 deg
    %
    % 边界设计原则：
    %   以 LS 初值为中心，给出足够大的绝对搜索余量，
    %   保证真实解一定落在边界范围内，同时防止优化跑偏。
    %
    %   距离边界：±50000 m（对 20000 m 偏差量级足够宽松）
    %   方位边界：±10 deg（对 3-4 deg 偏差量级足够覆盖）
    %
    % 为什么不使用 ±50% 的相对边界？
    %   1. LS 本身就是最优估计，不需要额外加 50% 的松动
    %   2. 当偏差接近 0 时，±50% 会导致边界收缩过窄（如 dr1_ls=0.1，边界仅 ±0.05m）
    %      优化稳定性会受影响，甚至可能把真值排除在边界外
    %   3. 使用较大的绝对余量更稳健

    % DIST_MARGIN：距离偏差搜索余量（米），在 LS 估计值上下各搜索 50000 米
    DIST_MARGIN = 50000;

    % AZI_MARGIN：方位偏差搜索余量（度），在 LS 估计值上下各搜索 10 度
    AZI_MARGIN = 10;

    % lb：下界向量 (lower bound)，每个元素对应一个优化变量的最小值
    % ... 是 MATLAB 的续行符，表示当前行延续到下一行
    % [dr1_ls - DIST_MARGIN] = 雷达1距离偏置下界（米）
    % [da1_ls - AZI_MARGIN]  = 雷达1方位偏置下界（度）
    lb = [dr1_ls - DIST_MARGIN, da1_ls - AZI_MARGIN, ...   % 雷达1距离/方位下界
          dr2_ls - DIST_MARGIN, da2_ls - AZI_MARGIN];      % 雷达2距离/方位下界

    % ub：上界向量 (upper bound)
    ub = [dr1_ls + DIST_MARGIN, da1_ls + AZI_MARGIN, ...   % 雷达1距离/方位上界
          dr2_ls + DIST_MARGIN, da2_ls + AZI_MARGIN];      % 雷达2距离/方位上界

    %% 配置 fmincon 的优化选项
    % optimoptions('fmincon', ...) 创建 fmincon 专用的优化选项对象
    %   'Display', 'iter'：在每次迭代时输出迭代信息（Function value, step size 等）
    %   'MaxIterations', 100：最大迭代次数，超过此时还未收敛则强制停止
    %   'OptimalityTolerance', 1e-10：一阶最优条件的容忍度阈值
    %       当梯度的无穷范数 < 1e-10 时认为已达最优，停止迭代
    %       1e-10 是非常严格的收敛标准，确保优化非常精确
    options = optimoptions('fmincon', 'Display', 'iter', ...
                           'MaxIterations', 100, ...
                           'OptimalityTolerance', 1e-10);

    %% 调用 fmincon 求解有约束非线性最小化问题
    % fmincon 函数语法：
    %   x_opt = fmincon(fun, x0, A, b, Aeq, beq, lb, ub, nonlcon, options)
    %
    % 各参数含义：
    %   cost_fcn：目标函数句柄，fmincon 会反复调用 cost_fcn(x) 评估目标值
    %   x0：      优化变量的初始值 [dr1_ls, da1_ls, dr2_ls, da2_ls]
    %   []：      线性不等式约束矩阵 A（这里不需要线性约束，传 []）
    %   []：      线性不等式约束向量 b
    %   []：      线性等式约束矩阵 Aeq（不需要）
    %   []：      线性等式约束向量 beq
    %   lb：      变量下界
    %   ub：      变量上界
    %   []：      非线性约束函数句柄（不需要，传 []）
    %   options： 优化选项结构体（含 Display, MaxIterations, OptimalityTolerance）
    %
    % 返回值 x_opt 是最优点 [dr1_opt, da1_opt, dr2_opt, da2_opt]
    % fmincon 内部使用 SQP（Sequential Quadratic Programming）算法：
    %   1. 在当前点将非线性优化近似为 QP 子问题
    %   2. 解 QP 子问题得到搜索方向
    %   3. 线搜索确定步长
    %   4. 迭代直到满足收敛条件
    x_opt = fmincon(cost_fcn, x0, [], [], [], [], lb, ub, [], options);


    % 输出分隔线和优化结果，方便调试
    disp("--------------------------------------------------------------------------------xopt");
    %% format long 设置数值显示格式为高精度（15位有效数字），便于观察优化结果
    format long;   % 关键：打开高精度
    disp(x_opt);   % 显示 fmincon 的最优解

    %% 最终决策：放弃 fmincon 精化结果，直接使用 LS 估计
    % 作者经过调试发现：fmincon 的精化效果在实验中不如直接 LS 估计准确
    % 可能原因：
    %   1. 代价函数在 ECEF 空间中可能存在局部极小值
    %   2. 标校点的 ECEF 转换引入了椭球模型的简化误差
    %   3. 下采样损失了部分信息
    % 因此最终输出覆盖为 LS 估计值
    x_opt = x0;     % 经过调参，fmincon 效果不如 LS 效果好，采用 LS 结果

    %% ============================================================================
    %% 输出：打包最终估计结果为结构体
    %% ============================================================================
    % 将最终的四维偏差估计向量打包成结构体 est，便于调用方使用
    % 字段名使用有意义的英文缩写：
    %   dr1 = distance range radar1（雷达1距离偏置）
    %   da1 = delta azimuth radar1（雷达1方位偏置）
    %   dr2 = distance range radar2（雷达2距离偏置）
    %   da2 = delta azimuth radar2（雷达2方位偏置）
    est = struct('dr1', x_opt(1), 'da1', x_opt(2), ...
                 'dr2', x_opt(3), 'da2', x_opt(4));

end  % 函数 estimate_biases 结束
