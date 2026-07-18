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
    % 判断优先级：IMM > 自适应 > 基础
    % 首先检查是否存在 ukf_cv 字段（IMM 特有的嵌套结构）
    % isstruct 确保它是结构体而非空数组或其他类型
    if isfield(ukf, 'ukf_cv') && isstruct(ukf.ukf_cv)
        % IMM 类型: 内部包含两个 ukf_jichu 实例（CV 和 CT 模型）
        fh = @ukf_imm;
    % 检查是否为自适应类型
    % 条件1: filter_type == 'zishiying'（create 时显式标记）
    % 条件2: 存在 maneuver_active 或 suspect_counter 字段（init 后自动创建）
    elseif (isfield(ukf, 'filter_type') && strcmp(ukf.filter_type, 'zishiying')) ...
            || isfield(ukf, 'maneuver_active') || isfield(ukf, 'suspect_counter')
        % 自适应类型: filter_type 标记 或 有机动检测字段（init 之后）
        fh = @ukf_zishiying;
    else
        % 基础类型: 最简单的 UKF，无自适应无 IMM
        fh = @ukf_jichu;
    end

    % ---- 委托调用 ----
    % 将 action 和剩余参数转发给选定的后端函数
    % nargout 决定返回几个变量，tracker 可能只取部分返回值
    [varargout{1:nargout}] = fh(action, ukf, varargin{:});
end
