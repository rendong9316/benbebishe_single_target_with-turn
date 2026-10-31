function h_m = iono_h_field(params, lon, lat, t_sec)
%IONO_H_FIELD 真实虚高的空间-时间场（仅量测生成侧使用）
%
%   h_m = iono_h_field(params, lon, lat, t_sec)
%
%   在 iono_h(params,'true') 的常数虚高基础上叠加两个可选失配项，
%   用于构造"模型不完全正确"的 L2 场景（避免 inverse crime）：
%
%     1. 经向梯度（空间失配）
%        h += iono_lon_tilt_km_per_deg * (lon - iono_lon_ref_deg)
%        物理含义：不同方位穿过不同电离层区域，等效虚高随经度倾斜。
%        标校与评估都会看到这一项，单常数 h 的估计器无法完全拟合，
%        残差与目标经度相关。
%
%     2. 时间漂移（时间失配）
%        h += iono_drift_amp_km * sin(2*pi*(t - phase)/T)
%        物理含义：电离层昼夜变化。标校窗内取均值（t 传 NaN），
%        评估窗内按 t_sec 漂移，二者不同期 -> 跨时段残差（对应 B2）。
%
%   两项默认幅度为 0，此时与 iono_h(params,'true') 完全等价。
%   反演/滤波侧的标称值仍由 iono_h(params,'assumed',id) 给出，不受影响。

    if nargin < 4, t_sec = NaN; end
    if nargin < 3, lat = NaN; end
    if nargin < 2, lon = NaN; end

    h_km = iono_h(params, 'true') / 1000;

    tilt = getdef_(params, 'iono_lon_tilt_km_per_deg', 0);
    if tilt ~= 0 && isfinite(lon)
        ref = getdef_(params, 'iono_lon_ref_deg', 107.4);
        h_km = h_km + tilt * (lon - ref);
    end

    amp = getdef_(params, 'iono_drift_amp_km', 0);
    if amp ~= 0 && isfinite(t_sec)
        T  = getdef_(params, 'iono_drift_period_h', 6) * 3600;
        ph = getdef_(params, 'iono_drift_phase_sec', 0);
        h_km = h_km + amp * sin(2*pi*(t_sec - ph)/T);
    end

    h_m = h_km * 1000;
end

function v = getdef_(s, name, def)
    if isfield(s, name) && ~isempty(s.(name)) && isfinite(double(s.(name)))
        v = double(s.(name));
    else
        v = def;
    end
end
