%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% based on current state, to predict the next step (by Kalman Filter for
% good tracks, or by associated point for temporary tracks)
%
% Input:
% curTrack: current track, the track structure, with no (or invalid)
% predict value.
% sysPara: system parameter, a structure, using the position of radar to 
% convert the predict value (in geo-coordinate) into radar coordinate (as
% the assocation is done in radar system)
% 
% Output:
% curTrack: the input track with predict value filled in. 
% ----------------------------------------------------------------------
% Date: 2022-03-31
% Author : Jun @ HIT
% ----------------------------------------------------------------------
% Modification: velocity ambiguity has been considered
% Date: 2023-04-02
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function curTrack = predictNextStep_cv(curTrack, sysPara, trackPara)

if curTrack.BatchNo == 20001
     disp(1);
end
% using sliding window to predict the next step 
winLen_vr = trackPara.prdct_v_winLen; % 120s means 4 points
winLen_az = trackPara.prdct_a_winLen; % 120s means 4 points, with largest error about 1.03 degree 
winLen_range = trackPara.prdct_r_winLen; % 180s means 5 points

% get useful information
assc_frameID = [curTrack.asscPointList(:).frameID];
assc_time = [curTrack.asscPointList(:).time];
assc_prange = [curTrack.asscPointList(:).prange];
assc_pvr = [curTrack.asscPointList(:).pvr];
assc_paz = [curTrack.asscPointList(:).paz];
smth_time = [curTrack.smoothPointList(:).time];
smth_range = [curTrack.smoothPointList(:).prange];
smth_pvr = [curTrack.smoothPointList(:).pvr];
pd_range =  [curTrack.asscPointList(end).pd_range];
pd_az =  [curTrack.asscPointList(end).pd_az];

% predict next state
next_paz = predictNext_azimuth_avg(assc_paz, winLen_az);
% next_pvr = predictNext_vr_avg(assc_time, assc_pvr, sysPara, winLen_vr);
next_pvr = predictNext_vr_avg(smth_time, smth_pvr, sysPara, winLen_vr);
% next_prange = predictNext_range_avg(assc_time, assc_prange, sysPara, next_pvr, winLen_range); 
next_prange = predictNext_range_avg(smth_time, smth_range, sysPara, next_pvr, winLen_range); 
next_daz = next_paz + pd_az;
next_dvr = next_pvr/pd_range;
next_drange = next_prange/pd_range;
% disp([next_range - mean(assc_range)])

% assign the structure
cnt = curTrack.TotalPointCnt + 1; % as you predict the next position, the predict value is one step ahead the track
curTrack.predictRes(cnt).prange = next_prange; %km
curTrack.predictRes(cnt).paz = next_paz; % deg
curTrack.predictRes(cnt).pvr = next_pvr; % m/s
curTrack.predictRes(cnt).drange = next_drange; %km
curTrack.predictRes(cnt).daz = next_daz; % deg
curTrack.predictRes(cnt).dvr = next_dvr; % m/s

% DONG_202512_v1
nextBLH = tool_radar2blh_fake_monostatic(sysPara.tx_BLH, sysPara.rx_BLH,  next_drange,  next_daz);
curTrack.predictRes(cnt).lat = nextBLH(1);
curTrack.predictRes(cnt).lon = nextBLH(2);
curTrack.predictRes(cnt).v_x = 0;
curTrack.predictRes(cnt).v_y = 0;

curTrack.predictRes(cnt).frameID = sysPara.frameID; % deg
curTrack.predictRes(cnt).time = sysPara.datenum; % 

end

% ---------- sub functions -------------------
function next_az = predictNext_azimuth_avg(assc_az, winLen_az)
% using mean value as the predict value
if length(assc_az) <= winLen_az
    next_az = median(assc_az);
    % next_az = median(assc_az);
else
    next_az = median(assc_az(end-winLen_az+1:end));
    % next_az = median(assc_az(end-winLen_az+1:end));
end

end

function next_vr = predictNext_vr_avg(timeList, vrList, sysPara, winLen_vr)
run('header.m'); 

% get the necessary data
if length(vrList) <= winLen_vr
    ind = 1:length(vrList);
else
    ind = length(vrList)-winLen_vr+1:length(vrList); 
    % next_vr = mean(assc_vr(end-winLen_vr+1:end));
end

% estimate the slope and the inception
ref_vr = median(vrList);
ref_time = median(timeList); 
timeDiff = tool_get_time_difference(timeList, ref_time, MATLAB_TIME_IN_SEC);
[~, kv] = robustMinSquareErr(timeDiff, vrList);
% prdc_vr = ref_vr + kv * timeDiff; 
deltaT = tool_get_time_difference(sysPara.datenum, ref_time, MATLAB_TIME_IN_SEC);
next_vr = ref_vr + kv * deltaT;

end

function next_range = predictNext_range_avg(timeList, rangeList, sysPara, next_vr, winLen_range)
run('header.m'); 

% get the necessary data
if length(rangeList) <= winLen_range
    ind = 1:length(rangeList);
else
    ind = length(rangeList)-winLen_range+1:length(rangeList); 
end

% find the weighted sum
% frameID is x
time_diff = tool_get_time_difference(timeList(ind), timeList(1), MATLAB_TIME_IN_SEC); % the order of each points w.r.t. the first one
cur_time = tool_get_time_difference(sysPara.datenum, timeList(1), MATLAB_TIME_IN_SEC); % the order of predicted points w.r.t. the first one
% y is the range
rr = rangeList(ind); 
% predict next range : considering the velocity ambiguity 
% true_vr = next_vr + ambgNum * sysPara.lambda / sysPara.prt;
true_vr = next_vr; 
next_range = mean(rr) - (cur_time - mean(time_diff))* true_vr/1e3;
if abs((cur_time - mean(time_diff))* true_vr/1e3) > 150
    warning(['predictNext_range: fly to far: time difference:', ...
        num2str(cur_time - mean(time_diff)), 'vr:', num2str(true_vr), ...
        'm/s, total range:', num2str(next_range-mean(rr)), 'km']); 
    next_range = mean(rr);
end
end