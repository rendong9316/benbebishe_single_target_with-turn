%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% sub-function for PointTrackAssociation_JNN
% calculate the cost of a point-track pair
%
% input
% track: interested track
% point: interested point
% sysPara: system parameter
% 
% output:
% pair_cost: the cost between the interesed track and the interested point
% ---------------------------------------------------------------------
% Author: Jun Geng
% Date: 2023 Jan. 7th
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function pair_cost = calculate_cost_of_point_track_pair(point, track, sysPara)

weight_range = 1; 
weight_vr = 0.5;
weight_theta = 0.01;

if isempty(point)
    % calculate the cost of false alarm
    % the cost of false alarm is assumed the point is on the boundary of 
    % the neighborhood of the track. hence the definition here must be 
    % consitent with the neighborhood of the track
    maxSuccLossCnt = 5; 
    sigmaR = track.filterPara.fixedR + maxSuccLossCnt * sysPara.T_inter * track.filterPara.floatR; 
    sigmaA = track.filterPara.fixedA + maxSuccLossCnt * sysPara.T_inter * track.filterPara.floatA; 
    sigmaV = track.filterPara.fixedV + maxSuccLossCnt * sysPara.T_inter * track.filterPara.floatV; 
    
    range_diff = sigmaR;
    az_diff = sigmaA;
    vr_diff = sigmaV;
else
    % calcuate the cost of normal pair
    range_diff = point.prange - track.predictRes(end).prange;
    az_diff = point.paz - track.predictRes(end).paz;
    vr_diff = point.pvr - track.predictRes(end).pvr;
end

% calculate the distance between the track and the point. Note that the
% distance should be normalized resolution
pair_cost = weight_range * abs(range_diff/sysPara.deltaR).^2 + ...
    weight_theta * abs(az_diff/sysPara.deltaAz).^2 +...
    weight_vr * abs(vr_diff/sysPara.deltaV).^2; 