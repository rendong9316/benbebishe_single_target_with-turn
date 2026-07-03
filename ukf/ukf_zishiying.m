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
    switch action
        case 'create'
            ukf = ukf_jichu('create', varargin{:});
            ukf.filter_type = 'zishiying';  % 标记，供 ukf_dispatch 路由
            varargout{1} = ukf;
        case 'init'
            ukf = ukf_jichu('init', varargin{:});
            % 初始化机动检测字段
            ukf.maneuver_active = false;
            ukf.maneuver_counter = 0;
            ukf.maneuver_recovery = 0;
            ukf.suspect_counter = 0;
            ukf.innov_history = {};
            ukf.last_det_list = [];
            varargout{1} = ukf;
        case 'prepare'
            % 委托 ukf_jichu，prepare 不含机动逻辑
            [varargout{1}, varargout{2}, varargout{3}, varargout{4}, ...
             varargout{5}, varargout{6}, varargout{7}] = ukf_jichu('prepare', varargin{:});
        case 'update'
            % varargin = {ukf, innov_w} — innov_w=[] 表示纯预测
            [lon, lat, ukf] = ukf_jichu('update', varargin{1}, varargin{2});
            if ~isempty(varargin{2})
                ukf.last_innov = varargin{2};  % 记录本帧新息供下帧机动检测
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
function ukf = apply_maneuver_adapt_post(ukf)
    params = ukf.params;
    ukf = adapt_q(ukf, params, 'zishiying');
end
