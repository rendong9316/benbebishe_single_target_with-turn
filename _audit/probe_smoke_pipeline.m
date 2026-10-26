% 冒烟测试：确认既有量测生成链路在 skywave_geometry 修改后仍可运行
clear; clc;
rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(rootDir,'utils')); addpath(fullfile(rootDir,'simulation'));
addpath(fullfile(rootDir,'config')); addpath(fullfile(rootDir,'ukf'));
p = simulation_params_oracle();
rng(11,'twister');
nDet = 0; nTgt = 0;
for k = 1:20
    [dl, hd] = generate_frame_detections(p.radar1_lon, p.radar1_lat, ...
        p.radar1_tx_lon, p.radar1_tx_lat, 118.5 + 0.02*k, 31.0 + 0.02*k, 0.002, 0.002, ...
        k, 30*k, p.radar1_range_bias_m, p.radar1_azimuth_bias_deg, ...
        p.radar1_beam_center_deg, p);
    nDet = nDet + numel(dl); nTgt = nTgt + hd;
end
fprintf('SMOKE detections=%d target_frames=%d\n', nDet, nTgt);
if nDet > 0
    fprintf('SMOKE first prange=%.1f km paz=%.2f deg\n', dl(1).prange/1e3, dl(1).paz);
end
fprintf('SMOKE %s\n', 'DONE');
