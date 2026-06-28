%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% created a new track by a group associated points
%
% Input:
% candidateTrack: a group of assoicated points
% sysPara: system parameter
% 
% Output:
% newTrack: with a group of associated points being assigned. 
% ----------------------------------------------------------------------
% Date: 2025-09-07
% Author : Jun @ HIT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function newTrack = fun_create_new_track(candidateTrack, sysPara)

header; 

asscPoints = candidateTrack.asscPointList;
asscPtsNum = length(asscPoints); 

%% fill out the common parts -- has to be modified in future
% the current informaiton of this track, should be aligned with the 22th
% information
newTrack.time = asscPoints(end).time; 
newTrack.BatchNo = [];

newTrack.lat = asscPoints(end).lat;  % deg
newTrack.lon = asscPoints(end).lon;  % deg
newTrack.v_x = 0;  % km % to remove in future
newTrack.v_y = 0;  % km % to remove in future
newTrack.sog = 0;  % km % to remove in future
newTrack.cog = 0;  % km % to remove in future
newTrack.az = asscPoints(end).daz; % deg % to remove in future
newTrack.range = asscPoints(end).drange; %km % to remove in future
newTrack.vr = asscPoints(end).dvr; % m/s  % here is the measured vr % to remove in future

%% fillout the point lists
% associate point list 
asscPointList = fun_fillout_assc_point_list(candidateTrack); 
newTrack.asscPointList = asscPointList;

% predictRes
prctPointList = fun_fillout_predict_point_list(candidateTrack); 
newTrack.predictRes = prctPointList;

% smooth point list
smoothPointList = fun_fillout_smooth_point_list(candidateTrack, sysPara); 
newTrack.smoothPointList = smoothPointList;
newTrack.outputPointList = smoothPointList; % m/s

%% fillout the filtering paramter
% filterPara
% revised by Jun @ Jan. 14, 2023, allow different track has different
% association zone, to make the program more flexible. 
% Part I: filter para
if abs(newTrack.vr) > MIN_RADIAL_VELOCITY
    % larger than 600km/h
    newTrack.filterPara.fixedR = FIXED_R_RADIUS_RADIAL_FLIGHT; %sigmaR;
    newTrack.filterPara.fixedA = FIXED_A_RADIUS_RADIAL_FLIGHT; %sigmaA;
    newTrack.filterPara.fixedV = FIXED_V_RADIUS_RADIAL_FLIGHT; %sigmaV;
    newTrack.filterPara.floatR = FLOAT_R_RADIUS_RADIAL_FLIGHT; %sigmaR;
    newTrack.filterPara.floatA = FLOAT_A_RADIUS_RADIAL_FLIGHT; %sigmaA;
    newTrack.filterPara.floatV = FLOAT_V_RADIUS_RADIAL_FLIGHT; %sigmaV;
else
    % for most of flight
    newTrack.filterPara.fixedR = FIXED_R_RADIUS_NORMAL_FLIGHT; %sigmaR;
    newTrack.filterPara.fixedA = FIXED_A_RADIUS_NORMAL_FLIGHT; %sigmaA;
    newTrack.filterPara.fixedV = FIXED_V_RADIUS_NORMAL_FLIGHT; %sigmaV;
    newTrack.filterPara.floatR = FLOAT_R_RADIUS_NORMAL_FLIGHT; %sigmaR;
    newTrack.filterPara.floatA = FLOAT_A_RADIUS_NORMAL_FLIGHT; %sigmaA;
    newTrack.filterPara.floatV = FLOAT_V_RADIUS_NORMAL_FLIGHT; %sigmaV;
end
newTrack.filterPara.stateNoise = diag([0.7, 0.7, 0.001, 0.001]); % km, km, km/s, km/s
newTrack.filterPara.obsNoise = diag([1, 1/180*pi]); % km, rad
newTrack.filterPara.state = [];
newTrack.filterPara.covar = [];
newTrack.filterPara.cog = nan; 
        
%% fillout the statisitcal information
newTrack.isNewTrack = 1; 
newTrack.updateFlag = 0; % added in 2023-04-11, to mark whether the current track has been updated (associated with a new point) or not 
newTrack.Type = TEMPORARY_TRACK;
newTrack.TotalPointCnt = length(smoothPointList);
newTrack.AsscPointCnt = length(asscPointList);
newTrack.TotalLostPointCnt = newTrack.TotalPointCnt - newTrack.AsscPointCnt;
newTrack.SuccLossPointCnt = 0; 
newTrack.Quality = NEW_TRACK_QUALITY;
newTrack.travelLen = 0; 
newTrack.travelLen = fun_calculate_track_travelLen(newTrack); 
newTrack.Region = 0; % added in 2024-01-03, to allow different processing parameter for different regions

end

% ------------ subfunctions --------------------------------------------
function asscPointList = fun_fillout_assc_point_list(candidateTrack) 

asscPoints = candidateTrack.asscPointList;
ptsNum = length(asscPoints); 
for pp = 1:ptsNum
    asscPointList(pp).frameID = asscPoints(pp).frameID;
    asscPointList(pp).time = asscPoints(pp).time;
    asscPointList(pp).ionoMode = asscPoints(pp).ionoMode;
    asscPointList(pp).prange = asscPoints(pp).prange;
    asscPointList(pp).paz = asscPoints(pp).paz;
    asscPointList(pp).pvr = asscPoints(pp).pvr;
    asscPointList(pp).drange = asscPoints(pp).drange;
    asscPointList(pp).daz = asscPoints(pp).daz;
    asscPointList(pp).dvr = asscPoints(pp).dvr;
    asscPointList(pp).pd_range = asscPoints(pp).pd_range;
    asscPointList(pp).pd_az = asscPoints(pp).pd_az;
    asscPointList(pp).lat = asscPoints(pp).lat;    
    asscPointList(pp).lon = asscPoints(pp).lon;
    asscPointList(pp).Rbin = asscPoints(pp).Rbin;
    asscPointList(pp).Dbin = asscPoints(pp).Dbin;
    asscPointList(pp).Abin = asscPoints(pp).Abin;
    asscPointList(pp).snr = asscPoints(pp).snr;
    asscPointList(pp).amp = asscPoints(pp).amp;
    asscPointList(pp).beampattern = asscPoints(pp).beampattern;
    asscPointList(pp).channvalue = asscPoints(pp).channvalue;
    asscPointList(pp).ambgNum = asscPoints(pp).ambgNum;
end
end

% MAYBE YOU CAN FIND A BETTER WAY TO PREDICT THE EXISTING TRACKS
function prctPointList = fun_fillout_predict_point_list(candidateTrack)

asscPoints = candidateTrack.asscPointList;
asscFrameIDList = [asscPoints(:).frameID]; 
minFrameID = asscFrameIDList(1); 
maxFrameID = asscFrameIDList(end);

% use interp to find the predict points
timeList = interp1(asscFrameIDList, [asscPoints(:).time], minFrameID:maxFrameID, 'linear'); 
latList = interp1(asscFrameIDList, [asscPoints(:).lat], minFrameID:maxFrameID, 'linear'); % km    % DONG_202512_v1
lonList = interp1(asscFrameIDList, [asscPoints(:).lon], minFrameID:maxFrameID, 'linear'); % km    % DONG_202512_v1
pazList = interp1(asscFrameIDList, [asscPoints(:).paz], minFrameID:maxFrameID, 'linear'); % deg
prangeList = interp1(asscFrameIDList, [asscPoints(:).prange], minFrameID:maxFrameID, 'linear'); %km
pvrList = interp1(asscFrameIDList, [asscPoints(:).pvr], minFrameID:maxFrameID, 'linear'); % m/s
dazList = interp1(asscFrameIDList, [asscPoints(:).daz], minFrameID:maxFrameID, 'linear'); % deg
drangeList = interp1(asscFrameIDList, [asscPoints(:).drange], minFrameID:maxFrameID, 'linear'); %km
dvrList = interp1(asscFrameIDList, [asscPoints(:).dvr], minFrameID:maxFrameID, 'linear'); % m/s

for curFrameID = minFrameID : maxFrameID
    % the index of current fillout position
    curIndex = curFrameID - minFrameID + 1; 
   
    prctPointList(curIndex).frameID = curFrameID;  % km
    prctPointList(curIndex).time = timeList(curIndex);  % km
    prctPointList(curIndex).lat = latList(curIndex);  % km % DONG_202512_v1
    prctPointList(curIndex).lon = lonList(curIndex);  % km % DONG_202512_v1
    prctPointList(curIndex).v_x = 0;
    prctPointList(curIndex).v_y = 0;
    prctPointList(curIndex).paz = pazList(curIndex); % deg
    prctPointList(curIndex).prange = prangeList(curIndex); %km
    prctPointList(curIndex).pvr = pvrList(curIndex); % m/s
    prctPointList(curIndex).daz = dazList(curIndex); % deg
    prctPointList(curIndex).drange = drangeList(curIndex); %km
    prctPointList(curIndex).dvr = dvrList(curIndex); % m/s
end
end

% smooth point list
% MAYBE YOU CAN FIND A BETTER WAY TO SMOOTH THE EXISTING TRACKS
function smoothPointList = fun_fillout_smooth_point_list(candidateTrack, sysPara) 
run('header.m');

% get the date for smoothing
asscPoints = candidateTrack.asscPointList;
asscFrameIDList = [asscPoints(:).frameID]; 
asscPD_range = [asscPoints(:).pd_range];
asscPD_az = [asscPoints(:).pd_az];

% set the frame ID for the smooth point list
minFrameID = asscFrameIDList(1); 
maxFrameID = asscFrameIDList(end);
smoothFrameList = minFrameID : maxFrameID; 

% get reference points for smoothing
asscTimeList = [asscPoints(:).time];
ref_time = median(asscTimeList);
ref_range = median([asscPoints(:).prange]);
ref_vr = median([asscPoints(:).pvr]);
ref_az = median([asscPoints(:).paz]); 
smoothTimeList = interp1(asscFrameIDList, asscTimeList, smoothFrameList, 'linear'); 
smoothPDrange = interp1(asscFrameIDList, asscPD_range, smoothFrameList, 'linear');
smoothPDaz = interp1(asscFrameIDList, asscPD_az, smoothFrameList, 'linear'); 

% get predict value using linear model
asscTimeDiff = tool_get_time_difference(asscTimeList, ref_time, MATLAB_TIME_IN_SEC);
smoothTimeDiff = tool_get_time_difference(smoothTimeList, ref_time, MATLAB_TIME_IN_SEC);
prdct_range = ref_range - ref_vr * smoothTimeDiff/1e3; % in km
[~, kv] = robustMinSquareErr(asscTimeDiff, [asscPoints(:).pvr]);
prdct_vr = ref_vr + kv * smoothTimeDiff; 
[~, ka] = robustMinSquareErr(asscTimeDiff, [asscPoints(:).paz]);
dA2 = abs(min(kv, 0) /ref_range/1000); % delta_theta square
da = sqrt(dA2)/pi*180; disp([num2str([ka, da]), 'deg/s']); 
ka = 0.5 * ka + 0.5 * sign(ka)*da; 
prdct_az = ref_az + ka * smoothTimeDiff; 

% fill the smooth result
weight_r = 0.75; weight_v = 0.7; weight_a = 0.85; 
for curFrameID = minFrameID : maxFrameID
    % the index of current fillout position
    curIndex = curFrameID - minFrameID + 1; 
    
    smoothPointList(curIndex).frameID = curFrameID; 
    smoothPointList(curIndex).time = smoothTimeList(curIndex); 
    
    ptsLocation = find(asscFrameIDList == curFrameID); 
    if ~isempty(ptsLocation)
        % Step 2: for the ones with associated points, the smooth result is a
        % weighted sum of the mean value and measured value
        smoothPointList(curIndex).prange = weight_r * prdct_range(curIndex) + (1 - weight_r) * asscPoints(ptsLocation).prange;
        smoothPointList(curIndex).pvr = weight_v * prdct_vr(curIndex) + (1-weight_v) * asscPoints(ptsLocation).pvr;
        smoothPointList(curIndex).paz = weight_a * prdct_az(curIndex) + (1-weight_a) * asscPoints(ptsLocation).paz;
        smoothPointList(curIndex).asscFlag = 1;
    else
        % Step 3: for the ones with unassociated points, the smooth result is the
        % interpation result
        smoothPointList(curIndex).prange = prdct_range(curIndex);
        smoothPointList(curIndex).pvr = prdct_vr(curIndex);
        smoothPointList(curIndex).paz = prdct_az(curIndex);
        smoothPointList(curIndex).asscFlag = 0;
    end
    smoothPointList(curIndex).drange = smoothPointList(curIndex).prange/smoothPDrange(curIndex);
    smoothPointList(curIndex).dvr = smoothPointList(curIndex).pvr/smoothPDrange(curIndex);
    smoothPointList(curIndex).daz = smoothPointList(curIndex).paz + smoothPDaz(curIndex);

    tgt_BLH = tool_radar2blh_fake_monostatic(sysPara.tx_BLH, sysPara.rx_BLH,   smoothPointList(curIndex).drange,  smoothPointList(curIndex).daz);
    smoothPointList(curIndex).v_x = 0;  % km
    smoothPointList(curIndex).v_y = 0;  % km
    smoothPointList(curIndex).lat = tgt_BLH(1);  % deg
    smoothPointList(curIndex).lon = tgt_BLH(2);  % deg
end

% % use interp to get the smooth points
% pazList = interp1(asscFrameIDList, [asscPoints(:).paz], smoothFrameList, 'linear'); % deg
% prangeList = interp1(asscFrameIDList, [asscPoints(:).prange], smoothFrameList, 'linear'); %km
% pvrList = interp1(asscFrameIDList, [asscPoints(:).pvr], smoothFrameList, 'linear'); % m/s
% dazList = interp1(asscFrameIDList, [asscPoints(:).daz], smoothFrameList, 'linear'); % deg
% drangeList = interp1(asscFrameIDList, [asscPoints(:).drange], smoothFrameList, 'linear'); %km
% dvrList = interp1(asscFrameIDList, [asscPoints(:).dvr], smoothFrameList, 'linear'); % m/s
% 
% % smoothing
% % Step 1: find the mean value of all associated points
% mean_prr = mean([asscPoints(:).prange]); 
% mean_pvr = mean([asscPoints(:).pvr]);
% mean_paz = mean([asscPoints(:).paz]);
% mean_drr = mean([asscPoints(:).drange]); 
% mean_dvr = mean([asscPoints(:).dvr]);
% mean_daz = mean([asscPoints(:).daz]);
% weight = 0.75;
% for curFrameID = minFrameID : maxFrameID
%     % the index of current fillout position
%     curIndex = curFrameID - minFrameID + 1; 
%     
%     smoothPointList(curIndex).frameID = curFrameID; 
%     smoothPointList(curIndex).time = timeList(curIndex); 
%     
%     ptsLocation = find(asscFrameIDList == curFrameID); 
%     if ~isempty(ptsLocation)
%         % Step 2: for the ones with associated points, the smooth result is a
%         % weighted sum of the mean value and measured value
%         smoothPointList(curIndex).prange = weight * mean_prr + (1-weight) * prangeList(curIndex);
%         smoothPointList(curIndex).pvr = weight * mean_pvr + (1-weight) * pvrList(curIndex);
%         smoothPointList(curIndex).paz = weight * mean_paz + (1-weight) * pazList(curIndex);
%         smoothPointList(curIndex).drange = weight * mean_drr + (1-weight) * drangeList(curIndex);
%         smoothPointList(curIndex).dvr = weight * mean_dvr + (1-weight) * dvrList(curIndex);
%         smoothPointList(curIndex).daz = weight * mean_daz + (1-weight) * dazList(curIndex);
%         smoothPointList(curIndex).asscFlag = 1;
%     else
%         % Step 3: for the ones with unassociated points, the smooth result is the
%         % interpation result
%         smoothPointList(curIndex).prange = prangeList(curIndex);
%         smoothPointList(curIndex).pvr = pvrList(curIndex);
%         smoothPointList(curIndex).paz = pazList(curIndex);
%         smoothPointList(curIndex).drange = drangeList(curIndex);
%         smoothPointList(curIndex).dvr = dvrList(curIndex);
%         smoothPointList(curIndex).daz = dazList(curIndex);
%         smoothPointList(curIndex).asscFlag = 0;
%     end
%     
%     % DONG_202512_v1
%     tgt_BLH = tool_radar2blh_fake_monostatic(sysPara.tx_BLH, sysPara.rx_BLH,   smoothPointList(curIndex).drange,  smoothPointList(curIndex).daz);
%     smoothPointList(curIndex).v_x = 0;  % km
%     smoothPointList(curIndex).v_y = 0;  % km
%     smoothPointList(curIndex).lat = tgt_BLH(1);  % deg
%     smoothPointList(curIndex).lon = tgt_BLH(2);  % deg
% end
end
