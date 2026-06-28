% constant definition
run('tool_header.m')
%% system working mode
% These variables correspond to the operating states of the system, such 
% as fast-scan mode, staring mode, sea-surveillance mode, and so on.
SYSTEM_WORKING_MODE1 = 1; 
SYSTEM_WORKING_MODE2 = 2;
SYSTEM_WORKING_MODE3 = 3;
SYSTEM_WORKING_MODE4 = 4;
SYSTEM_WORKING_MODE5 = 5;

%% track quality Control
% track quality ID
RELIABLE_TRACK = 1; 
MAINTAIN_TRACK = 2;
GOOD_TRACK = 3;
INSPECTING_TRACK = 4;
TEMPORARY_TRACK = 6;
HISTORY_TRACK = 7;

% ---Start: Marcos in fun_track_quality_management_and_info_completion ---
% assocation bonus
ASSOCIATION_AWARD = 1; % used in 
POINT_LOSS_FOR_NEW_TRACK = 1;
POINT_LOSS_FOR_GOOD_TRACK = 2;

% quality control
% Track Termination Conditions
% 1. Maximum Number of Consecutive Missing Points = (QUALITY_MAX -
% QUALITY_MIN)/POINT_LOSS_FOR_GOOD_TRACK; 
% 2. Allowable Number of Missing Plots for New Tracks = (NEW_TRACK_QUALITY 
% - QUALITY_MIN)/POINT_LOSS_FOR_NEW_TRACK
% 3. Confirmed Track Length = TRACK_STARTER_M + (QUALITY_RELIABLE - 
% NEW_TRACK_QUALITY)/ASSOCIATION_AWARD
QUALITY_MIN = 5; 
QUALITY_MAX = 15;
QUALITY_RELIABLE = 10; 
NEW_TRACK_QUALITY = 8;
% ---End: Marcos in fun_track_quality_management_and_info_completion ---

%% track association parameter
% ----------------Start: Marcos in fun_create_new_track ----------------
% used in calculate_cost_of_point_track_pair
% used in determine_if_point_within_the_scope_of_track
FIXED_R_RADIUS_NORMAL_FLIGHT = 60; %km 34
FIXED_V_RADIUS_NORMAL_FLIGHT = 7.5; %m/s 8.4
FIXED_A_RADIUS_NORMAL_FLIGHT = 9; % deg 7
FLOAT_R_RADIUS_NORMAL_FLIGHT = 0.1; %km/s 12
FLOAT_V_RADIUS_NORMAL_FLIGHT = 0.02; %m/s2 1.2
FLOAT_A_RADIUS_NORMAL_FLIGHT = 0.02; % deg/s 0.4

FIXED_R_RADIUS_RADIAL_FLIGHT = 60; % 42
FIXED_V_RADIUS_RADIAL_FLIGHT = 10; % 8.4*1.35
FIXED_A_RADIUS_RADIAL_FLIGHT = 9; % 7;
FLOAT_R_RADIUS_RADIAL_FLIGHT = 0.1; % 15
FLOAT_V_RADIUS_RADIAL_FLIGHT = 0.03; % 1.4*1.35;
FLOAT_A_RADIUS_RADIAL_FLIGHT = 0.01; % 0.1
% ----------------End: Marcos in fun_create_new_track ---------------

%% track starter
TRACK_STARTER_LOGIC_M = 5; 
TRACK_STARTER_LOGIC_N = 9; 

% the definition of neighboorhood
% ---------- begin: fun_find_best_asscpoints_NN -------------------------
% the scope of neighboorhood
% 【天波超视距雷达适配】
% 天波雷达群距离受电离层几何放大，不同帧间群距离变化可达数百km。
% 逐维门限（Range/Az/Vr）对天波场景不适用，仅依赖归一化综合距离。
% 将各维门限设为大值以跳过逐维检查，实际筛选由 NN_OVERALL 完成。
NN_RANGE_RADIUS = 5000;  % the allowable distance between  km（逐维检查已禁用）
NN_VR_RADIUS = 500;      % m/s（逐维检查已禁用）
NN_AZ_RADIUS = 180;      % deg（逐维检查已禁用）
% the parmaeter to calculate the distance between the measured points and
% the predict result; the higher the weight is, the more accurate (less
% bias or narrower scope) the corresponding dimension is;
NN_WEIGHT_R = 1;
NN_WEIGHT_V = 1;
NN_WEIGHT_A = 0.2;
NN_OVERALL = 40;       % 归一化综合距离门限（实际起作用的唯一门限）
% ---------- end: fun_find_best_asscpoints_NN -------------------------


MIN_TRACK_ASSC_LEN = 5; % number of points
MIN_TRACK_TIME_LEN = 3; % min

% track report
MIN_REPORT_LEN = 7; 

ASSC_POINT_MAX = 200;

% maximum time interval
MAX_WAITING_TIME = 200; % max allowed waiting time 

% for radial target
MIN_RADIAL_VELOCITY = 200 * 2; % m/s 400m/s <-> 720km/h

%% sub-regions in RD plane
% for track start, sub-region contain less tracks and points, which
% accelerate the processing speed
range1 = 1600; % km
range2 = 2100; % km
range3 = 2600; % km
range4 = 3100; % km
range5 = 3600; % km
range6 = 4100; % km

RDZONE(1, 1:4) = [-inf, range1, -inf, inf]; 
RDZONE(2, 1:4) = [range1, range3, -inf, inf];
RDZONE(3, 1:4) = [range3, range5, -inf, inf];
RDZONE(4, 1:4) = [range5, inf, -inf, inf];
[ZONE_NUM,~ ]= size(RDZONE); 
GUARD_RANGE = 50; % km
GUARD_VR = 7; % m/s



%% sub-regions in geo-coordinate
% define regions
R1 = 1488.4; theta1 = 104.65; % km, n2e, deg
R2 = 1553.8; theta2 = 109.63; % km, n2e, deg
R3 = 2035.0; theta3 = 95.0; % km, n2e, deg
R4 = 1987.3; theta4 = 90.3; % km, n2e, deg
Region1.name = 'SouthJapan';
Region1.p1_xoy = [R1*sind(theta1), R1*cosd(theta1)]; % left top
Region1.p2_xoy = [R2*sind(theta2), R2*cosd(theta2)]; % left bottom
Region1.p3_xoy = [R3*sind(theta3), R3*cosd(theta3)]; % right bottom
Region1.p4_xoy = [R4*sind(theta4), R4*cosd(theta4)]; % right top
Region1.p1_bhl = [132.4821, 34.7587];
Region1.p2_bhl = [132.4804, 33.4453];
Region1.p3_bhl = [139.2257, 35.4184];
Region1.p4_bhl = [139.2280, 36.9223];
Region1.CogNum = 2; 
Region1.Cog(1).mean = 255.5; % in deg 
Region1.Cog(1).ci = [254, 257]; % in deg
Region1.Cog(1).fd = [0, 380]; % in m/s
Region1.Cog(2).mean = 78.5;  
Region1.Cog(2).ci = [72, 80]; 
Region1.Cog(2).fd = [-380, 0];
% Region1.CogHypothsis(3).mean = 270; % mean value of direction 
% Region1.CogHypothsis(3).ci = [268, 272]; % confidence interval
% Region1.CogHypothsis(3).fd = [0, inf]; % the suitable doppler zone


R1 = 885.0; theta1 = 107.14; % km, n2e, deg
R2 = 1040.0; theta2 = 120.62; % km, n2e, deg
R3 = 1130.0; theta3 = 112.80; % km, n2e, deg
R4 = 1000.0; theta4 = 103.06; % km, n2e, deg
Region2.name = 'WestKorean';
Region2.p1_xoy = [R1*sind(theta1), R1*cosd(theta1)]; % left top
Region2.p2_xoy = [R2*sind(theta2), R2*cosd(theta2)]; % left bottom
Region2.p3_xoy = [R3*sind(theta3), R3*cosd(theta3)]; % right bottom
Region2.p4_xoy = [R4*sind(theta4), R4*cosd(theta4)]; % right top
Region2.p1_bhl = [126.12, 36.3];
Region2.p2_bhl = [126.12, 34.38];
Region2.p3_bhl = [127.36, 34.7];
Region2.p4_bhl = [127.36, 36.3];
Region2.CogNum = 2; 
Region2.Cog(1).mean = 179; % in deg 
Region2.Cog(1).ci = [170, 190]; % in deg
Region2.Cog(1).fd = [-inf, 0]; % in m/s
Region2.Cog(2).mean = 4;  
Region2.Cog(2).ci = [0, 8]; 
Region2.Cog(2).fd = [0, inf];


Region9.name = 'JapanSea';
Region9.p1_xoy = [1050.0, -70.0]; % left top
Region9.p2_xoy = [1050.0, -300.0]; % left bottom
Region9.p3_xoy = [1350.0, -300.0]; % right bottom
Region9.p4_xoy = [1350.0, -150.0]; % right top
Region9.p1_bhl = [128.05, 37.61];
Region9.p2_bhl = [128.05, 35.65];
Region9.p3_bhl = [131.41, 35.65];
Region9.p4_bhl = [131.41, 37.15];
Region9.CogNum = 4; 
Region9.Cog(1).mean = 108; % in deg 
Region9.Cog(1).ci = [101, 114]; % in deg
Region9.Cog(1).fd = [-inf, 0]; % in m/s
Region9.Cog(2).mean = 294;  
Region9.Cog(2).ci = [292, 296]; 
Region9.Cog(2).fd = [0, inf];
Region9.Cog(3).mean = 270;  
Region9.Cog(3).ci = [275, 280]; 
Region9.Cog(3).fd = [0, inf];
Region9.Cog(4).mean = 315;  
Region9.Cog(4).ci = [210, 230]; 
Region9.Cog(4).fd = [0, inf];