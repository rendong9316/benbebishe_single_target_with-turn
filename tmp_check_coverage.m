addpath(genpath('.'));
params = simulation_params();

fprintf('=== Final waypoint search ===\n');

% Strategy: fix A/B endLon=130.0 -> 1624s, 55 frames, full coverage
% Then make C also 1624s by adjusting latitude or path

% C: straight east at lat 32.5, lon 126.8->131.0 gives 1712s
% Need C to be 1624s. Distance for 1624s at 230m/s = 373.5 km
% At lat 32.5, 1 deg lon = 111*cos(32.5) = 93.7 km
% So need 373.5/93.7 = 3.99 deg lon span -> 126.8+3.99 = 130.79

wpC = [126.8, 32.5, 0; 130.8, 32.5, 0];
trC = aircraft_trajectory_create(wpC, params.aircraft_speed_ms, params.dt_sec);
ttC = aircraft_trajectory_interpolate('generate', trC);
n1C=0; n2C=0;
for j=1:size(ttC,1)
    [in1,~,~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ttC(j,1), ttC(j,2), params.radar1_beam_center_deg, params);
    [in2,~,~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ttC(j,1), ttC(j,2), params.radar2_beam_center_deg, params);
    if in1, n1C=n1C+1; end; if in2, n2C=n2C+1; end
end
fprintf('C: lon 126.8->130.8 dur=%.0fs (%.0f frames) R1=%d/%d R2=%d/%d\n', ...
    trC.duration_sec, floor(trC.duration_sec/params.dt_sec)+1, n1C,size(ttC,1), n2C,size(ttC,1));

% A: 126.8->130.0, lat 31.5->33.5
wpA = [126.8, 31.5, 0; 130.0, 33.5, 0];
trA = aircraft_trajectory_create(wpA, params.aircraft_speed_ms, params.dt_sec);
ttA = aircraft_trajectory_interpolate('generate', trA);
n1A=0; n2A=0;
for j=1:size(ttA,1)
    [in1,~,~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ttA(j,1), ttA(j,2), params.radar1_beam_center_deg, params);
    [in2,~,~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ttA(j,1), ttA(j,2), params.radar2_beam_center_deg, params);
    if in1, n1A=n1A+1; end; if in2, n2A=n2A+1; end
end
fprintf('A: dur=%.0fs (%.0f frames) R1=%d/%d R2=%d/%d\n', ...
    trA.duration_sec, floor(trA.duration_sec/params.dt_sec)+1, n1A,size(ttA,1), n2A,size(ttA,1));

% B: 126.8->130.0, lat 33.5->31.5
wpB = [126.8, 33.5, 0; 130.0, 31.5, 0];
trB = aircraft_trajectory_create(wpB, params.aircraft_speed_ms, params.dt_sec);
ttB = aircraft_trajectory_interpolate('generate', trB);
n1B=0; n2B=0;
for j=1:size(ttB,1)
    [in1,~,~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ttB(j,1), ttB(j,2), params.radar1_beam_center_deg, params);
    [in2,~,~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ttB(j,1), ttB(j,2), params.radar2_beam_center_deg, params);
    if in1, n1B=n1B+1; end; if in2, n2B=n2B+1; end
end
fprintf('B: dur=%.0fs (%.0f frames) R1=%d/%d R2=%d/%d\n', ...
    trB.duration_sec, floor(trB.duration_sec/params.dt_sec)+1, n1B,size(ttB,1), n2B,size(ttB,1));
