%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: 
% When no plots are associated with tracks, use the track predicted 
% values as the smoothing values.
% Input: 
% curTrack: current Track with no smoothing 
% Output:
% curTrack: current Track with smoothing fields being updated
% ---------------------------------------------------------------------
% Author: Jun Geng
% Date: 2026-01-24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function curTrack = fun_fill_smooth_list_by_predict_result(curTrack, sysPara)

trackLen = length(curTrack.predictRes);
drange = curTrack.predictRes(trackLen).drange;
daz = curTrack.predictRes(trackLen).daz;

curTrack.smoothPointList(trackLen).time = sysPara.datenum;
curTrack.smoothPointList(trackLen).frameID = sysPara.frameID;

curTrack.smoothPointList(trackLen).prange = curTrack.predictRes(trackLen).prange; %km
curTrack.smoothPointList(trackLen).paz = curTrack.predictRes(trackLen).paz; % deg
curTrack.smoothPointList(trackLen).pvr = curTrack.predictRes(trackLen).pvr; % m/s
curTrack.smoothPointList(trackLen).drange = curTrack.predictRes(trackLen).drange; %km
curTrack.smoothPointList(trackLen).daz = curTrack.predictRes(trackLen).daz; % deg
curTrack.smoothPointList(trackLen).dvr = curTrack.predictRes(trackLen).dvr; % m/s
curTrack.smoothPointList(trackLen).v_x = curTrack.predictRes(trackLen).v_x;  % 
curTrack.smoothPointList(trackLen).v_y = curTrack.predictRes(trackLen).v_y;  % 

tgt_BLH = tool_radar2blh_fake_monostatic(sysPara.tx_BLH, sysPara.rx_BLH, drange,  daz);
curTrack.smoothPointList(trackLen).lat = tgt_BLH(1);  % km
curTrack.smoothPointList(trackLen).lon = tgt_BLH(2);  % km

curTrack.smoothPointList(trackLen).asscFlag = 0;


end