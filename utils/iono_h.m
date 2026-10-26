function h_m = iono_h(params, which, radar_id)
%IONO_H 电离层等效虚高的统一取值入口（单位：米）
%
%   h_m = iono_h(params, 'true')            量测生成用（仿真真值）
%   h_m = iono_h(params, 'assumed')         反演/滤波/标定用的标称值
%   h_m = iono_h(params, 'assumed', id)     指定雷达（id=1/2）的标称值
%
%   设计意图：
%     本课题的时空配准要解决的核心误差源是"电离层虚高失配"——
%     真实传播的等效虚高（约 320 km）与系统假定的标称值（300 km）不同，
%     导致群距离偏差随目标距离变化（近距约 20 km，远距约 11 km），
%     无法用常数偏差补偿。
%
%     因此必须把"真值"和"标称值"彻底分开：
%       - generate_frame_detections_multi 生成量测时用 'true'
%       - bistatic_inverse_solver / ukf_jichu 反演与滤波时用 'assumed'
%       - calibrate_bias 标定时用 'assumed'（模拟"不知道真值"的工程现实）
%
%   向后兼容：
%     若 params 中没有 ionosphere_* 字段，两者都返回 300 km，
%     与旧版硬编码行为完全一致。
%
%   2026-10-26

    if nargin < 2 || isempty(which), which = 'assumed'; end
    if nargin < 3, radar_id = []; end

    switch lower(which)
        case 'true'
            h_km = getdef(params, 'ionosphere_true_h_km', 300.0);
        case 'assumed'
            % 优先取雷达专属的标称值（配准后可被更新），其次取全局标称值
            h_km = [];
            if ~isempty(radar_id)
                h_km = getdef(params, sprintf('radar%d_assumed_h_km', radar_id), []);
            end
            if isempty(h_km)
                h_km = getdef(params, 'ionosphere_assumed_h_km', 300.0);
            end
        otherwise
            error('iono_h:badMode', 'which 必须是 ''true'' 或 ''assumed''，收到 %s', which);
    end

    h_m = h_km * 1000.0;
end

% ---------------------------------------------------------------
function v = getdef(s, name, def)
    if isfield(s, name) && ~isempty(s.(name)) && isfinite(double(s.(name)))
        v = double(s.(name));
    else
        v = def;
    end
end
