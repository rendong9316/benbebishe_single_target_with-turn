% =========================================================================
% ukf_dispatch.m — UKF 滤波器多态路由
% =========================================================================
% 【功能概述】
%   根据 ukf 结构体内部特征字段，自动路由到对应的滤波器实现：
%     ukf.ukf_cv 存在       → ukf_imm     (IMM: CV+CT 双模型)
%     ukf.maneuver_active   → ukf_zishiying (机动自适应 UKF)
%     以上均不存在           → ukf_jichu    (基础 UKF)
%
% 【统一接口】
%   ukf = ukf_dispatch('create',  params, radar_lon, radar_lat, tx_lon, tx_lat, dt)
%   ukf = ukf_dispatch('init',    ukf, meas1, meas2)
%   [x_pred, z_pred, P_zz, ukf] = ukf_dispatch('prepare', ukf)
%       注: prepare 完整返回 7 个输出，tracker 通过 ~ 忽略多余项
%   [lon, lat, ukf] = ukf_dispatch('update',  ukf, innov_w)
%
% 【设计原则】
%   tracker 只调用 ukf_dispatch，不感知后端滤波器的具体类型。
%   主入口注入哪种 ukf_tpl（由对应 create 产生），tracker 自动走对应路径。
% =========================================================================

function varargout = ukf_dispatch(action, ukf, varargin)
    % ---- 根据 ukf 内部特征字段选择后端 ----
    if isfield(ukf, 'ukf_cv') && isstruct(ukf.ukf_cv)
        % IMM 类型: 内部包含两个 ukf_jichu 实例
        fh = @ukf_imm;
    elseif (isfield(ukf, 'filter_type') && strcmp(ukf.filter_type, 'zishiying')) ...
            || isfield(ukf, 'maneuver_active') || isfield(ukf, 'suspect_counter')
        % 自适应类型: filter_type 标记 或 有机动检测字段（init 之后）
        fh = @ukf_zishiying;
    elseif isfield(ukf, 'filter_type') && strcmp(ukf.filter_type, 'imm_3in1')
        % 三合一 IMM 类型: 复用 ukf_imm
        fh = @ukf_imm;
    else
        % 基础类型
        fh = @ukf_jichu;
    end

    % ---- 委托调用 ----
    [varargout{1:nargout}] = fh(action, ukf, varargin{:});
end
