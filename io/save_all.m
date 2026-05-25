% ============================================================================
% save_all.m
% 数据持久化模块 —— 将仿真结果保存为 CSV / MAT / JSON 三种格式
% ============================================================================
%
% 【功能概述】
%   将一次仿真运行的全部结果（真实航迹、两部雷达的量测数据、仿真参数）
%   保存到指定输出目录。输出三种互补格式：
%     - CSV（Comma-Separated Values）：人类可读的表格文本格式，
%       可用 Excel、Python pandas 等工具直接打开和绘图
%     - MAT（MATLAB 原生二进制格式）：供 MATLAB 后续分析脚本
%       直接 load 使用，加载速度快，保留完整数值精度
%     - JSON（JavaScript Object Notation）：元数据和仿真参数，
%       纯文本、跨平台、可人工阅读，记录实验配置用于复现
%
% 【在空间配准流程中的角色】
%   本函数是整个仿真管线的最后一步——数据持久化。在仿真计算完成后：
%     1. 将真实航迹输出为 true_track.csv（含经纬度、速度、时间）
%     2. 将两部雷达量测分别输出为 radar1_measurements.csv 和
%        radar2_measurements.csv（含量测值、真实值、经纬度）
%     3. 将所有数值变量打包保存到 simulation_data.mat
%     4. 将仿真元参数保存到 simulation_metadata.json
%
%   此函数等价于 Python 版本的 simulation/io.py。
%
% 【文件格式说明】
%
%   CSV 格式:
%     true_track.csv 列：time_str, lon_deg, lat_deg, lon_rate_dps,
%                        lat_rate_dps, speed_ms
%     radar*_measurements.csv 列：time_str, range_meas_m,
%                        azimuth_meas_deg, radial_vel_meas_ms,
%                        range_true_m, azimuth_true_deg,
%                        radial_vel_true_ms, lat_deg, lon_deg
%
%   MAT 格式:
%     变量列表：true_track, r1_time, r1_range_meas, r1_az_meas,
%              r1_rv_meas, r1_range_true, r1_az_true, r1_rv_true,
%              r1_lat, r1_lon, r2_*（雷达2对应字段）
%
%   JSON 格式:
%     字段：project, phase, generated_at, ref_start_time, params
%     params 子字段：duration_sec, dt_sec, radar坐标, 噪声标准差,
%                    检测概率, 随机种子等
%
% 【输入参数】
%   true_track     - 真实航迹数组，大小为 (n_steps, 5) 或 (n_steps, 7)
%                    列含义：
%                      列1: lon       - 经度（度）
%                      列2: lat       - 纬度（度）
%                      列3: lon_rate  - 经度变化率（度/秒）
%                      列4: lat_rate  - 纬度变化率（度/秒）
%                      列5: time      - 时间（秒）
%                    注：有些代码中列3可能是填充的0
%   r1_meas_list   - 雷达1量测序列，类型为 cell array of struct
%                    每个 struct 包含量测字段和真实字段
%   r2_meas_list   - 雷达2量测序列，类型为 cell array of struct
%                    格式同 r1_meas_list
%   params         - 仿真参数结构体，包含：
%                      ref_start_time: 参考起始时间（datetime）
%                      duration_sec:   仿真时长（秒）
%                      dt_sec:         时间步长（秒）
%                      radar1_lon/lat: 雷达1部署坐标
%                      radar2_lon/lat: 雷达2部署坐标
%                      range_noise_std_m: 距离噪声标准差（米）
%                      azimuth_noise_std_deg: 方位噪声标准差（度）
%                      radial_vel_noise_std_ms: 径向速度噪声标准差（m/s）
%                      detection_probability: 检测概率
%                      random_seed: 随机种子
%   out_dir        - 输出目录路径，类型为 char 字符串
%                    如果目录不存在则自动创建
%
% 【返回值】
%   无（void 函数，副作用是创建文件）
%
% 【调用关系】
%   本函数调用：extract_measurement_field（提取量测字段）
%              sphere_utils_get_earth_radius（获取地球半径）
%              sphere_utils_seconds_to_datetime_str（秒数转时间字符串）
%   本函数被调用：main.m 或仿真主控脚本（最后一步）
%
% ============================================================================

function save_all(true_track, r1_meas_list, r2_meas_list, params, out_dir)
    % 保存全部仿真数据到指定目录（CSV + MAT + JSON 三格式输出）
    %
    % 输入:
    %   true_track:   真实航迹数组 (n_steps, 5)
    %                 列：[lon, lat, lon_rate, lat_rate, time]
    %   r1_meas_list: 雷达1量测 cell array of struct
    %   r2_meas_list: 雷达2量测 cell array of struct
    %   params:       simulation_params 结构体
    %   out_dir:      输出目录路径字符串

    %% ---- 确保输出目录存在 ----
    % exist(out_dir, 'dir') 检查 out_dir 是否存在且为目录
    % ~ 是逻辑非：如果不存在，返回 true
    % mkdir(out_dir) 创建目录
    % 这样即使 out_dir 的父目录已存在，也能安全创建子目录
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    %% ---- 获取航迹行数 ----
    n = size(true_track, 1);   % size(X, 1) 返回矩阵的行数，即时间步数

    %% ====================================================================
    %% 第一部分：保存真实航迹 CSV 文件（true_track.csv）
    %% ====================================================================
    % 文件内容包括：时间字符串、经纬度、经纬度变化率、合速度（米/秒）
    %
    % 速度计算原理：
    %   经纬度变化率（度/秒）需要转换为地面速度（米/秒）
    %   设经度变化率为 lon_rate（度/秒），纬度变化率为 lat_rate（度/秒）
    %   地球半径 R（米），当前纬度 lat（弧度）
    %
    %   东向速度：v_east = lon_rate * (π/180) * R * cos(lat)
    %     说明：经度1度对应地面弧长 = (π/180) * R * cos(lat)
    %     原因：经线在赤道处最宽（每度约111km），向两极收敛（极点处每度距离为0）
    %
    %   北向速度：v_north = lat_rate * (π/180) * R
    %     说明：纬度1度始终对应约111km（地球近似为球体时）
    %
    %   合速度：speed = sqrt(v_east^2 + v_north^2)

    % 打开文件（写入模式）
    % fullfile(out_dir, 'true_track.csv') 拼接路径，自动处理操作系统的路径分隔符
    fid = fopen(fullfile(out_dir, 'true_track.csv'), 'w');

    % 写入 CSV 表头
    % dps = degrees per second（度/秒）
    fprintf(fid, 'time_str,lon_deg,lat_deg,lon_rate_dps,lat_rate_dps,speed_ms\n');

    % 获取地球半径（米），用于速度转换
    R = 6371000.0;

    % 逐行写入真实航迹数据
    for i = 1:n
        %% 将当前纬度的经度变化率（度/秒）转换为东向速度（米/秒）
        % lat_rad: 当前纬度（弧度）
        % cos(lat_rad): 纬度余弦因子，经线在该纬度处的"收缩系数"
        lat_rad = deg2rad(true_track(i, 2));

        % v_east: 东向速度 = 经度变化率 × 每度弧长
        % (pi/180) * R: 球面上每度对应的弧长（约111 km/度）
        % * cos(lat_rad): 经线在纬度为lat处的收缩因子
        v_east  = true_track(i, 3) * (pi/180) * R * cos(lat_rad);

        % v_north: 北向速度 = 纬度变化率 × 每度弧长
        % 纬度每度弧长始终为 (pi/180) * R（在球体近似下与纬度无关）
        v_north = true_track(i, 4) * (pi/180) * R;

        % 合速度：勾股定理合成东向和北向分量
        speed_ms = sqrt(v_east^2 + v_north^2);

        % 将秒偏移转换为可读时间字符串（格式：YYYY-MM-DD HH:MM:SS）
        time_str = sphere_utils_seconds_to_datetime_str(true_track(i, 5), params.ref_start_time);

        % 写入一行 CSV 数据
        % 格式说明符：%s=字符串, %.8f=双精度浮点(8位小数), %.6f=浮点(6位小数)
        % \n 是换行符（Windows 上 fopen 会自动转为 \r\n）
        fprintf(fid, '%s,%.8f,%.8f,%.6f,%.6f,%.6f\n', ...
                time_str, ...
                true_track(i, 1), true_track(i, 2), ...   % 经纬度（度）
                true_track(i, 3), true_track(i, 4), ...   % 经纬度变化率（度/秒）
                speed_ms);                                  % 合速度（米/秒）
    end
    fclose(fid);   % 关闭文件，确保缓冲区写入磁盘

    %% ====================================================================
    %% 第二部分：保存两部雷达量测 CSV 文件
    %% ====================================================================
    % 分别为雷达1和雷达2生成独立的 CSV 文件
    % 使用一个 2×2 的 cell 数组 radars 来组织数据，避免代码重复：
    %   radars{1,1} = 'radar1', radars{1,2} = r1_meas_list
    %   radars{2,1} = 'radar2', radars{2,2} = r2_meas_list
    radars = {'radar1', r1_meas_list; 'radar2', r2_meas_list};

    for r = 1:2
        %% 取出当前雷达的名称和量测数据
        rname = radars{r, 1};     % 雷达名称字符串（'radar1' 或 'radar2'）
        rmeas = radars{r, 2};     % 量测数据 cell array

        %% 打开输出文件
        % sprintf('%s_measurements.csv', rname) 生成文件名
        % 例如 rname = 'radar1' → 'radar1_measurements.csv'
        fid = fopen(fullfile(out_dir, sprintf('%s_measurements.csv', rname)), 'w');

        %% 写入表头
        % 各列含义：
        %   time_str:            时间字符串
        %   range_meas_m:        量测距离（米，含系统偏置和噪声）
        %   azimuth_meas_deg:    量测方位角（度，含系统偏置和噪声）
        %   radial_vel_meas_ms:  量测径向速度（米/秒）
        %   range_true_m:        真实距离（米，从真实航迹计算的理想值）
        %   azimuth_true_deg:    真实方位角（度）
        %   radial_vel_true_ms:  真实径向速度（米/秒）
        %   lat_deg:             目标纬度（度）
        %   lon_deg:             目标经度（度）
        % [...] 是 MATLAB 的字符串拼接语法（拼接成一行）
        fprintf(fid, ['time_str,range_meas_m,azimuth_meas_deg,radial_vel_meas_ms,' ...
                      'range_true_m,azimuth_true_deg,radial_vel_true_ms,lat_deg,lon_deg\n']);

        %% 逐帧写入量测数据
        for i = 1:length(rmeas)
            m = rmeas{i};   % 用花括号读取 cell 数组中第 i 帧量测 struct

            %% 处理漏检帧：写入一行 nan 和空逗号
            % 漏检帧（空数组）只需要标记时间字符串，其余字段留空或填 nan
            if isempty(m)
                fprintf(fid, 'nan,,,,,,,,\n');
                continue;   % 跳过当前帧，处理下一帧
            end

            %% 处理有效帧：根据 struct 中是否包含量测字段分情况处理
            % 对齐后的数据可能不含量测字段（只有插值出的经纬度），
            % 也可能包含完整的量测和真实值字段
            if isfield(m, 'range_meas')
                %% 情况A：量测帧（含完整的量测值）
                % 写入：时间、量测距离、量测方位、量测径向速度
                fprintf(fid, '%s,%.3f,%.6f,%.4f', m.time_str, ...
                        m.range_meas, m.azimuth_meas, m.radial_vel_meas);

                % 如果有真实值字段（量测生成时计算的真值），一并写入
                if isfield(m, 'range_true')
                    fprintf(fid, ',%.3f,%.6f,%.4f', ...
                            m.range_true, m.azimuth_true, m.radial_vel_true);
                else
                    % 无真实值时用逗号占位
                    fprintf(fid, ',,,');
                end

                % 最后写入目标经纬度并换行
                fprintf(fid, ',%.8f,%.8f\n', m.lat, m.lon);

            else
                %% 情况B：对齐后插值帧（只含经纬度和时间，无雷达量测信息）
                % 只写入时间字符串和经纬度，雷达相关字段全部留空
                fprintf(fid, '%s,,,,,,,,%.8f,%.8f\n', m.time_str, m.lat, m.lon);
            end
        end
        fclose(fid);   % 关闭当前雷达的 CSV 文件
    end  % for r 循环结束——两部雷达的 CSV 均已写入

    %% ====================================================================
    %% 第三部分：保存 MAT 文件（MATLAB 原生二进制格式）
    %% ====================================================================
    % MAT 文件用于后续 MATLAB 分析脚本直接 load
    % 优点：加载速度快（二进制格式），保留完整数值精度（IEEE 754 双精度）
    % 使用 extract_measurement_field 工具函数从 cell array 中提取各字段
    %
    % 提取策略：
    %   对每个字段调用一次 extract_measurement_field，返回 N×1 列向量
    %   extract_measurement_field 会自动将漏检帧填 NaN

    %% 雷达1 量测字段提取
    r1_time       = extract_measurement_field(r1_meas_list, 'time_sec');
    r1_range_meas = extract_measurement_field(r1_meas_list, 'range_meas');
    r1_az_meas    = extract_measurement_field(r1_meas_list, 'azimuth_meas');
    r1_rv_meas    = extract_measurement_field(r1_meas_list, 'radial_vel_meas');
    r1_range_true = extract_measurement_field(r1_meas_list, 'range_true');
    r1_az_true    = extract_measurement_field(r1_meas_list, 'azimuth_true');
    r1_rv_true    = extract_measurement_field(r1_meas_list, 'radial_vel_true');
    r1_lat        = extract_measurement_field(r1_meas_list, 'lat');
    r1_lon        = extract_measurement_field(r1_meas_list, 'lon');

    %% 雷达2 量测字段提取
    r2_time       = extract_measurement_field(r2_meas_list, 'time_sec');
    r2_range_meas = extract_measurement_field(r2_meas_list, 'range_meas');
    r2_az_meas    = extract_measurement_field(r2_meas_list, 'azimuth_meas');
    r2_rv_meas    = extract_measurement_field(r2_meas_list, 'radial_vel_meas');
    r2_range_true = extract_measurement_field(r2_meas_list, 'range_true');
    r2_az_true    = extract_measurement_field(r2_meas_list, 'azimuth_true');
    r2_rv_true    = extract_measurement_field(r2_meas_list, 'radial_vel_true');
    r2_lat        = extract_measurement_field(r2_meas_list, 'lat');
    r2_lon        = extract_measurement_field(r2_meas_list, 'lon');

    %% 保存 .mat 文件
    % save(filename, var1, var2, ...) 将指定变量保存到 MAT 文件
    % fullfile 拼接输出目录和文件名
    % 变量名即字段名，load 后会以相同变量名加载到工作区
    save(fullfile(out_dir, 'simulation_data.mat'), ...
         'true_track', ...                                   % 真实航迹 (n_steps × 5)
         'r1_time', 'r1_range_meas', 'r1_az_meas', 'r1_rv_meas', ...   % 雷达1量测
         'r1_range_true', 'r1_az_true', 'r1_rv_true', ...   % 雷达1真实值
         'r1_lat', 'r1_lon', ...                             % 雷达1目标经纬度
         'r2_time', 'r2_range_meas', 'r2_az_meas', 'r2_rv_meas', ...   % 雷达2量测
         'r2_range_true', 'r2_az_true', 'r2_rv_true', ...   % 雷达2真实值
         'r2_lat', 'r2_lon');                                % 雷达2目标经纬度

    %% ====================================================================
    %% 第四部分：保存 JSON 元数据文件
    %% ====================================================================
    % JSON 文件记录实验的配置参数和元信息，用于实验复现
    % 字段包括：项目名称、阶段、生成时间、参考时间、所有仿真参数

    meta = struct();   % 创建空结构体

    % 项目标识信息
    meta.project = 'HF Passive Radar Dual-Illuminator Track Association and Fusion';
    meta.phase = '1_trajectory_simulation';

    % 文件生成时间
    % datestr(now, 'yyyy-mm-dd HH:MM:SS') 返回当前时间的格式化字符串
    meta.generated_at = datestr(now, 'yyyy-mm-dd HH:MM:SS');

    % 参考起始时间（仿真时间轴的零点）
    meta.ref_start_time = datestr(params.ref_start_time, 'yyyy-mm-dd HH:MM:SS');

    % 仿真参数子结构体
    % 包含所有控制仿真行为的关键参数，记录在此以便日后复现
    meta.params = struct( ...
        'duration_sec', params.duration_sec, ...               % 仿真时长（秒）
        'dt_sec', params.dt_sec, ...                           % 仿真时间步长（秒）
        'radar1_lon', params.radar1_lon, 'radar1_lat', params.radar1_lat, ...  % 雷达1部署坐标
        'radar2_lon', params.radar2_lon, 'radar2_lat', params.radar2_lat, ...  % 雷达2部署坐标
        'range_noise_std_m', params.range_noise_std_m, ...                     % 距离噪声标准差（米）
        'azimuth_noise_std_deg', params.azimuth_noise_std_deg, ...             % 方位噪声标准差（度）
        'radial_vel_noise_std_ms', params.radial_vel_noise_std_ms, ...         % 径向速度噪声标准差（m/s）
        'detection_probability', params.detection_probability, ...             % 检测概率
        'random_seed', params.random_seed);                                    % 随机种子

    %% 写入 JSON 文件
    % jsonencode(meta, 'PrettyPrint', true) 将 MATLAB 结构体编码为 JSON 字符串
    %   'PrettyPrint', true: 启用缩进格式化，提高可读性（否则会输出为单行）
    fid = fopen(fullfile(out_dir, 'simulation_metadata.json'), 'w');
    fprintf(fid, '%s', jsonencode(meta, 'PrettyPrint', true));
    fclose(fid);

    %% ---- 完成提示 ----
    % 输出保存成功的确认信息到命令窗口
    fprintf('数据已保存到 %s/\n', out_dir);

end  % 函数 save_all 结束
