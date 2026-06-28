%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% sub-function for PointTrackAssociation_JNN
% determine if the interested point within the neighborhood of the 
% interested track
%
% input
% track: interested track
% point: interested point
% sysPara: system parameter
% 
% output:
% flag: bool_flag = 1 if the point within the neighborhood of the interested
% track; bool_flag = 0, otherwise. 
% ---------------------------------------------------------------------
% Author: Jun Geng
% Date: 2023 Jan. 7th
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function bool_flag = determine_if_point_within_the_scope_of_track(point, track, sysPara)

% get the size of neighborhood 
% sigmaR = track.filterPara.fixedR + track.SuccLossPointCnt * track.filterPara.floatR; 
% sigmaA = track.filterPara.fixedA + track.SuccLossPointCnt * track.filterPara.floatA; 
% sigmaV = track.filterPara.fixedV + track.SuccLossPointCnt * track.filterPara.floatV; 
[sigmaR, sigmaA, sigmaV] = get_tracking_gate(track.filterPara.fixedR, ...
    track.filterPara.fixedA, track.filterPara.fixedV, track.filterPara.floatR, ...
    track.filterPara.floatA, track.filterPara.floatV, track.SuccLossPointCnt, sysPara, 1); 

range_diff = point.prange - track.predictRes(end).prange; 
theta_diff = point.paz - track.predictRes(end).paz;
vr_diff = point.pvr - track.predictRes(end).pvr;

isRangeOK = (abs(range_diff) <= sigmaR);
isAzOK = (abs(theta_diff) <= sigmaA);
isVrOK = (abs(vr_diff) <= sigmaV);

bool_flag = (isRangeOK && isAzOK && isVrOK ); 
