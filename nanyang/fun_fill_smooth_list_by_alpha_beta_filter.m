%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% 1.smooth the track according to the assocate point and the predict result
% 2.fill out the field of smooth point list
% 
% Input:
% curTrack: current Track, with filled predict information
% asscPoint: associated point. if there has no association, put is as emety
% sysPara: system parameter
%
% Output:
% curTrack: current track, with the smoothed field being filled
% 
% Remark:
% Usually, this function should be complete by Kalman filter. However, for
% this type of radar, we reject the filtering method due to low measurement
% accuracy and large miss detection probablity 
% ------------------------------------------------------------------------
% Author: Jun Geng
% Date: 2023-04-10
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function curTrack = fun_fill_smooth_list_by_alpha_beta_filter(curTrack, asscPoint, sysPara)

% run('header.m')
header; 

% Input Paramter Check
if isempty(asscPoint)
    error('no association points! fun_fill_smooth_list_by_predict_result should be called');
end

% find the smooth result 
[smooth_range, smooth_vr, smooth_az, smooth_vx, smooth_vy] = fun_trackfilter_AlphaBeta(curTrack, asscPoint, sysPara);

tgt_BLH = tool_radar2blh_fake_monostatic(sysPara.tx_BLH, sysPara.rx_BLH, smooth_range,  smooth_az);
lat = tgt_BLH(1); 
lon = tgt_BLH(2); 

% fillout the smoothPointLsit
% 1.set the common fields
curTrack.smoothPointList(curTrack.TotalPointCnt).time = curTrack.time;
curTrack.smoothPointList(curTrack.TotalPointCnt).frameID = sysPara.frameID;
curTrack.smoothPointList(curTrack.TotalPointCnt).asscFlag = 1;    
    
% 2. fill out the other fields
curTrack.smoothPointList(curTrack.TotalPointCnt).prange = smooth_range; %km
curTrack.smoothPointList(curTrack.TotalPointCnt).paz = smooth_az; % deg
curTrack.smoothPointList(curTrack.TotalPointCnt).pvr = smooth_vr; % m/s
curTrack.smoothPointList(curTrack.TotalPointCnt).drange = smooth_range/curTrack.asscPointList(end).pd_range; %km
curTrack.smoothPointList(curTrack.TotalPointCnt).daz = smooth_az+curTrack.asscPointList(end).pd_az; % deg
curTrack.smoothPointList(curTrack.TotalPointCnt).dvr = smooth_vr/curTrack.asscPointList(end).pd_range; % m/s
curTrack.smoothPointList(curTrack.TotalPointCnt).v_x = smooth_vx;  % km
curTrack.smoothPointList(curTrack.TotalPointCnt).v_y = smooth_vy;  % km
curTrack.smoothPointList(curTrack.TotalPointCnt).lat = lat;  % km
curTrack.smoothPointList(curTrack.TotalPointCnt).lon = lon;  % km
end


