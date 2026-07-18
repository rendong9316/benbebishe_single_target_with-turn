% =========================================================================
% ukf_zishiying.m
% =========================================================================
% 功能概要：
%   自适应 UKF — 过程式调度器，封装 ukf_jichu + 自动自适应 Q。
%   'update' 动作接收外部准备好的新息和预测统计，内部委托
%   ukf_jichu('update', ...) 完成纯 Kalman 数学，再施加模糊自适应 Q。
%
% 公共 actions：
%   ukf = ukf_zishiying('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt)
%   ukf = ukf_zishiying('init', ukf_tpl, meas1, meas2)
%   [lon, lat, ukf] = ukf_zishiying('update', ukf, innov, z_pred, Z_pred, ...
%       X_pred, x_pred, P_pred, P_zz, params)
%      纯 Kalman 更新 + 机动自适应 Q（模糊自适应 + 机动检测 + Q 提升）
% =========================================================================

function varargout = ukf_zishiying(action, varargin)
    % 自适应 UKF — 过程式调度器
    % 封装 ukf_jichu + 自动自适应 Q 调整。
    % 在基础 UKF 的 Kalman 更新之后，额外施加模糊自适应 Q 和机动检测 Q 提升。
    %
    % 与 ukf_jichu 的区别：
    %   - 'update' action 接收更多参数（innov, z_pred, Z_pred, X_pred, x_pred, P_pred, P_zz, params）
    %   - 内部调用 ukf_jichu('update', ...) 完成纯 Kalman 数学
    %   - 更新后自动调用 adapt_q 施加自适应 Q
    %   - 维护机动检测状态字段（maneuver_active, maneuver_counter, suspect_counter）

    switch action
        case 'create'
            % 创建 UKF 模板：与 ukf_jichu 完全相同
            ukf = ukf_jichu('create', varargin{:});
            % 标记滤波器类型为 'zishiying'，供 ukf_dispatch 路由判断
            ukf.filter_type = 'zishiying';
            varargout{1} = ukf;

        case 'init'
            % 初始化：两点法初始化 UKF 状态 + 机动检测字段
            ukf = ukf_jichu('init', varargin{:});
            % 初始化机动检测状态机字段
            ukf.maneuver_active = false;   % 当前是否处于机动状态
            ukf.maneuver_counter = 0;      % 机动持续帧数
            ukf.maneuver_recovery = 0;     % 机动恢复帧数（连续无机动才递增）
            ukf.suspect_counter = 0;       % 可疑帧计数
            ukf.innov_history = {};        % 新息历史（供机动检测）
            ukf.last_det_list = [];        % 上次关联点迹列表
            varargout{1} = ukf;

        case 'prepare'
            % 预测步：委托 ukf_jichu，prepare 不含机动逻辑
            [varargout{1}, varargout{2}, varargout{3}, varargout{4}, ...
             varargout{5}, varargout{6}, varargout{7}] = ukf_jichu('prepare', varargin{:});

        case 'update'
            % 更新步：先执行 Kalman 更新，再施加自适应 Q
            % varargin = {ukf, innov_w} — innov_w=[] 表示纯预测
            [lon, lat, ukf] = ukf_jichu('update', varargin{1}, varargin{2});

            % 如果有新息（非纯预测帧），记录并施加自适应 Q
            if ~isempty(varargin{2})
                ukf.last_innov = varargin{2};  % 记录本帧新息供下帧机动检测
                % adapt_q 内部执行：模糊推理 + 机动检测 + EMA 平滑
                ukf = adapt_q(ukf, ukf.params, 'zishiying');
            end
            varargout = {lon, lat, ukf};

        otherwise
            error('ukf_zishiying: unknown action ''%s''', action);
    end
end


% =========================================================================
% apply_maneuver_adapt_post — 机动自适应后处理（委托给通用 adapt_q）
% =========================================================================
% 这是一个便捷函数，供外部在 tracker 主循环中手动调用
% 通常在 track 生命周期结束时调用，确保 Q 恢复到合理值
function ukf = apply_maneuver_adapt_post(ukf)
    params = ukf.params;
    ukf = adapt_q(ukf, params, 'zishiying');
end
