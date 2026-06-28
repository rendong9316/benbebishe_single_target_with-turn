%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% convert the detected point to the format required by the track processor
%
% Input:
% detPointList: all detected point from the detector
% frameID: curent frame ID
% time: current time
% 
% Output:
% trackPointList: all detected point in track processor required format
% ----------------------------------------------------------------------
% Date: 2022-03-31
% Author : Jun @ HIT
% ---------------------------------------------------------------------
% Modified by Jun @ 2023-04-40
% expand (copy) the detected points to cover the velocity ambiguity target
% --------------------------------------------------------------------
% Remarked by Jun @ 202512
% this program is only avaiable for filght target. No expansion is required
% for ship target
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function trackPointList = det2trackDataConverter(detPointList, pdInfo, sysPara)

working_mode = 1; % this should be replaced by some variable in system paramter
if working_mode == 1
    % for flight detection, has velocity ambiguity
    trackPointList = det2track_point_converter_for_fight(detPointList, pdInfo, sysPara); 
    
elseif working_mode == 2
    % for ship detection
    trackPointList = det2track_point_converter_for_ship(detPointList, pdInfo, sysPara); 
    
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: 
% convert the detected point into the points used in the tracking program.
% This program is for flight, flight has velocity ambiguity, hence the
% detected points has been extended to the case with velocity
% -------------------------------------------------------
% Author: Jun Geng @ 2025-12-28
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function trackPointList = det2track_point_converter_for_fight(detPointList, pdInfo, sysPara)

% step 0: remove all points within the clutter region
fb = 0.102*sqrt(sysPara.f0); %Hz
vb = fb * sysPara.lambda; 
clt_ind = find(abs([detPointList(:).vr]) < 7*vb);
detPointList(clt_ind) = [];

% setp 1: calculte the velocity boundary (with the ambiguity) 
Vmax_unamb = 0.5 * sysPara.lambda/sysPara.prt; % maximum unambiguity velocity of the system
Vmax_radial = 666; % maximum velocity of radial target
Vmax_amb = 2 * abs(sysPara.fIndex(1) * sysPara.lambda); % the maximum velocity that the system can achieves when ambiguitiy is allowed (we only allow ambiguity = 1).
Vmax_allow = min(Vmax_amb, Vmax_radial); 
V_cutoff = max(0, 2*Vmax_unamb - Vmax_allow); 

% copy the measured point
pointNum = length(detPointList); % total number of points
trackPointList = [];
for pp = 1:pointNum
    % for each point, convert it format into the required format
    % 1. set general information
    trackPointList(pp).frameID = detPointList(pp).frameID;
    trackPointList(pp).time = detPointList(pp).datenum;
    trackPointList(pp).ionoMode = 5; % MARK: should come from detected point 
    
    % 2. set the measured propagation information 
    trackPointList(pp).prange = detPointList(pp).range;
    trackPointList(pp).paz = detPointList(pp).az; % MARK
    trackPointList(pp).pvr = detPointList(pp).vr;
    
    % 3. set the ground distance information
    [drange, daz, dvr, pd_range, pd_az] = func_cal_gruond_distance_from_group_path(trackPointList(pp).prange,...
        trackPointList(pp).paz, trackPointList(pp).pvr, pdInfo, trackPointList(pp).ionoMode);
    trackPointList(pp).drange = drange;
    trackPointList(pp).daz = daz; 
    trackPointList(pp).dvr = dvr;
    trackPointList(pp).pd_range = pd_range; 
    trackPointList(pp).pd_az = pd_az;
    
    % 4. set the geometric information
    tgt_BLH = tool_radar2blh_fake_monostatic(sysPara.tx_BLH, sysPara.rx_BLH, drange,  detPointList(pp).az);
    trackPointList(pp).lat = tgt_BLH(1); 
    trackPointList(pp).lon = tgt_BLH(2); 
    
    % 5. set the debug/detection informaiton
    trackPointList(pp).Rbin = detPointList(pp).Rbin;
    trackPointList(pp).Dbin = detPointList(pp).Dbin;
    trackPointList(pp).Abin = detPointList(pp).Abin;
    trackPointList(pp).snr = detPointList(pp).SNR;
    trackPointList(pp).amp = detPointList(pp).Amp;
    trackPointList(pp).beampattern = detPointList(pp).beampattern;
    trackPointList(pp).channvalue = detPointList(pp).channvalue;
    trackPointList(pp).ambgNum = 0; % add a new member @ 2023-04-30
end

% newly added to solve the velocity ambiguity @ 2023-04-30
% expand the measured point by ambiguity 1 
ind1 = find([trackPointList(:).pvr] < -V_cutoff);
trackPointList_p = trackPointList(ind1);
pointNum = length(trackPointList_p); % total number of points
for pp = 1:pointNum
    % only revised the necessary component
    trackPointList_p(pp).pvr = trackPointList_p(pp).pvr + 2 * Vmax_unamb;
    trackPointList_p(pp).dvr = trackPointList_p(pp).pvr/trackPointList_p(pp).pd_range;
    trackPointList_p(pp).ambgNum = 1; 
end

% expand the measured point by ambiguity -1 
ind1 = find([trackPointList(:).pvr] > V_cutoff);
trackPointList_n = trackPointList(ind1);
pointNum = length(trackPointList_n); % total number of points
for pp = 1:pointNum
    % only revised the necessary component
    trackPointList_n(pp).pvr = trackPointList_n(pp).pvr - 2 * Vmax_unamb;
    trackPointList_n(pp).dvr = trackPointList_n(pp).pvr/trackPointList_n(pp).pd_range;
    trackPointList_n(pp).ambgNum = -1; 
end
% output
trackPointList = [trackPointList, trackPointList_p, trackPointList_n]; 

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: 
% convert the detected point into the points used in the tracking program.
% This program is for ship target, ship does not havevelocity ambiguity, 
% hence detected points are kept to be original
% -------------------------------------------------------
% Author: Jun Geng @ 2025-12-28
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function trackPointList = det2track_point_converter_for_ship(detPointList, pdInfo, sysPara)

% step 0: remove the points too large velocity
Vmax_unamb = 0.5 * sysPara.lambda/sysPara.prt; % maximum unambiguity velocity of the system
Vmax_radial = 50; % maximum velocity of radial target is 50kn
V_cutoff = min(Vmax_unamb, Vmax_radial); 

% setp 1: copy the measured point
ind1 = find(abs([detPointList(:).vr]) > V_cutoff);
detPointList(ind1) = []; % remove the points with too large velocity
pointNum = length(detPointList); % total number of points
for pp = 1:pointNum
    % for each point, convert it format into the required format
    % 1. set general information
    trackPointList(pp).frameID = detPointList(pp).frameID;
    trackPointList(pp).time = detPointList(pp).datenum;
    trackPointList(pp).ionoMode = 5; % MARK: should come from detected point
    
    % 2. set the measured propagation information 
    trackPointList(pp).prange = detPointList(pp).range;
    trackPointList(pp).paz = detPointList(pp).az; % MARK
    trackPointList(pp).pvr = detPointList(pp).vr;
    
    % 3. set the ground distance information
    [drange, daz, dvr, pd_range, pd_az] = func_cal_gruond_distance_from_group_path(trackPointList(pp).prange,...
        trackPointList(pp).paz, trackPointList(pp).pvr, pdInfo, trackPointList(pp).ionoMode);
    trackPointList(pp).drange = drange;
    trackPointList(pp).daz = daz; % MARK
    trackPointList(pp).dvr = dvr;
    trackPointList(pp).pd_range = pd_range; % MARK
    trackPointList(pp).pd_az = pd_az;
    
    % 4. set the geometric information
    tgt_BLH = tool_radar2blh_fake_monostatic(sysPara.tx_BLH, sysPara.rx_BLH, drange,  detPointList(pp).az);
    trackPointList(pp).lat = tgt_BLH(1); 
    trackPointList(pp).lon = tgt_BLH(2); 
    
    % 5. set the debug/detection informaiton
    trackPointList(pp).Rbin = detPointList(pp).Rbin;
    trackPointList(pp).Dbin = detPointList(pp).Dbin;
    trackPointList(pp).Abin = detPointList(pp).Abin;
    trackPointList(pp).snr = detPointList(pp).SNR;
    trackPointList(pp).amp = detPointList(pp).Amp;
    trackPointList(pp).beampattern = detPointList(pp).beampattern;
    trackPointList(pp).channvalue = detPointList(pp).channvalue;
    trackPointList(pp).ambgNum = 0; % add a new member @ 2023-04-30
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: 
% calcute the ground distance (from radar to traget) using Pd coefficient
% and measured group range
% ------------------------------------------------------------------------
% Author: Jun Geng
% Date: 2026-01-24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [drange, daz, dvr, pd_range, pd_az] = func_cal_gruond_distance_from_group_path(prange, paz, pvr, pdInfo, ionoMode)

% step 1: select the proper pd_coefficient
modeindex = ionoMode;
if modeindex == 1
    % Propagation Mode 1 corresponds to EE mode
    sec1_range_pd_index = pdInfo(1).rIndex; 
    sec1_pd_range = pdInfo(1).EE1_pdR;
    sec1_pd_az = pdInfo(1).EE1_pdAz; 
    sec2_range_pd_index = pdInfo(2).rIndex;
    sec2_pd_range = pdInfo(2).EE1_pdR;
    sec2_pd_az = pdInfo(2).EE1_pdAz;
    sec3_range_pd_index = pdInfo(3).rIndex;
    sec3_pd_range = pdInfo(3).EE1_pdR;
    sec3_pd_az = pdInfo(3).EE1_pdAz;
    sec4_range_pd_index = pdInfo(4).rIndex;
    sec4_pd_range = pdInfo(4).EE1_pdR;
    sec4_pd_az = pdInfo(4).EE1_pdAz;
    sec5_range_pd_index = pdInfo(5).rIndex;
    sec5_pd_range = pdInfo(5).EE1_pdR;
    sec5_pd_az = pdInfo(5).EE1_pdAz; 
elseif modeindex == 2
    sec1_range_pd_index = pdInfo(1).rIndex;
    sec1_pd_range = pdInfo(1).EF2_pdR;
    sec1_pd_az = pdInfo(1).EF2_pdAz;
    sec2_range_pd_index = pdInfo(2).rIndex;
    sec2_pd_range = pdInfo(2).EF2_pdR;
    sec2_pd_az = pdInfo(2).EF2_pdAz;
    sec3_range_pd_index = pdInfo(3).rIndex;
    sec3_pd_range = pdInfo(3).EF2_pdR;
    sec3_pd_az = pdInfo(3).EF2_pdAz;
    sec4_range_pd_index = pdInfo(4).rIndex;
    sec4_pd_range = pdInfo(4).EF2_pdR;
    sec4_pd_az = pdInfo(4).EF2_pdAz;
    sec5_range_pd_index = pdInfo(5).rIndex;
    sec5_pd_range = pdInfo(5).EF2_pdR;
    sec5_pd_az = pdInfo(5).EF2_pdAz; 
elseif modeindex == 3
    sec1_range_pd_index = pdInfo(1).rIndex;
    sec1_pd_range = pdInfo(1).FE3_pdR;
    sec1_pd_az = pdInfo(1).FE3_pdAz;
    sec2_range_pd_index = pdInfo(2).rIndex;
    sec2_pd_range = pdInfo(2).FE3_pdR;
    sec2_pd_az = pdInfo(2).FE3_pdAz;
    sec3_range_pd_index = pdInfo(3).rIndex;
    sec3_pd_range = pdInfo(3).FE3_pdR;
    sec3_pd_az = pdInfo(3).FE3_pdAz;
    sec4_range_pd_index = pdInfo(4).rIndex;
    sec4_pd_range = pdInfo(4).FE3_pdR;
    sec4_pd_az = pdInfo(4).FE3_pdAz;
    sec5_range_pd_index = pdInfo(5).rIndex;
    sec5_pd_range = pdInfo(5).FE3_pdR;
    sec5_pd_az = pdInfo(5).FE3_pdAz;
elseif modeindex == 4
    sec1_range_pd_index = pdInfo(1).rIndex;
    sec1_pd_range = pdInfo(1).FF4_pdR;
    sec1_pd_az = pdInfo(1).FF4_pdAz;
    sec2_range_pd_index = pdInfo(2).rIndex;
    sec2_pd_range = pdInfo(2).FF4_pdR;
    sec2_pd_az = pdInfo(2).FF4_pdAz;
    sec3_range_pd_index = pdInfo(3).rIndex;
    sec3_pd_range = pdInfo(3).FF4_pdR;
    sec3_pd_az = pdInfo(3).FF4_pdAz;
    sec4_range_pd_index = pdInfo(4).rIndex;
    sec4_pd_range = pdInfo(4).FF4_pdR;
    sec4_pd_az = pdInfo(4).FF4_pdAz;
    sec5_range_pd_index = pdInfo(5).rIndex;
    sec5_pd_range = pdInfo(5).FF4_pdR;
    sec5_pd_az = pdInfo(5).FF4_pdAz;
else
    sec1_range_pd_index = (500:100:2200)*2;
    sec1_pd_range = ones(1,size(sec1_range_pd_index,2));
    sec1_pd_az = zeros(1,size(sec1_range_pd_index,2));
    sec2_range_pd_index = (500:100:2200)*2;
    sec2_pd_range = ones(1,size(sec2_range_pd_index,2));
    sec2_pd_az = zeros(1,size(sec2_range_pd_index,2));
    sec3_range_pd_index = (500:100:2200)*2;
    sec3_pd_range = ones(1,size(sec3_range_pd_index,2));
    sec3_pd_az = zeros(1,size(sec3_range_pd_index,2));
    sec4_range_pd_index = (500:100:2200)*2;
    sec4_pd_range = ones(1,size(sec4_range_pd_index,2));
    sec4_pd_az = zeros(1,size(sec4_range_pd_index,2));
    sec5_range_pd_index = (500:100:2200)*2;
    sec5_pd_range = ones(1,size(sec5_range_pd_index,2));
    sec5_pd_az = zeros(1,size(sec5_range_pd_index,2));
end

% step 2: located current position and select propoer pd
curAz = paz; 
isInSector1 = (curAz > pdInfo(1).az_min) && (curAz <= pdInfo(1).az_max) ; 
isInSector2 = (curAz > pdInfo(2).az_min) && (curAz <= pdInfo(2).az_max) ; 
isInSector3 = (curAz > pdInfo(3).az_min) && (curAz <= pdInfo(3).az_max) ; 
isInSector4 = (curAz > pdInfo(4).az_min) && (curAz <= pdInfo(4).az_max) ; 
isInSector5 = (curAz > pdInfo(5).az_min) && (curAz <= pdInfo(5).az_max) ; 
if isInSector1
    range_pd_index = sec1_range_pd_index;
    pd_range = sec1_pd_range;
    pd_az = sec1_pd_az; 
elseif isInSector2
    range_pd_index = sec2_range_pd_index;
    pd_range = sec2_pd_range;
    pd_az = sec2_pd_az; 
elseif isInSector3
    range_pd_index = sec3_range_pd_index;
    pd_range = sec3_pd_range;
    pd_az = sec3_pd_az; 
elseif isInSector4
    range_pd_index = sec4_range_pd_index;
    pd_range = sec4_pd_range;
    pd_az = sec4_pd_az; 
elseif isInSector5
    range_pd_index = sec5_range_pd_index;
    pd_range = sec5_pd_range;
    pd_az = sec5_pd_az; 
else
    warning('det2trackDataConverter: Unexpected branch!'); 
    range_pd_index = (500:100:2200)*2;
    pd_range = ones(size(range_pd_index));
    pd_az = zeros(size(sec5_range_pd_index)); 
end
    
curRange = prange;
if curRange > range_pd_index(end)
    curPd_range = pd_range(end);
    curPd_az = pd_az(end); 
elseif curRange < range_pd_index(1)
    curPd_range = pd_range(1); 
    curPd_az = pd_az(1); 
else
    curPd_range = interp1(range_pd_index, pd_range, curRange, 'linear');
    curPd_az = interp1(range_pd_index, pd_az, curRange, 'linear');
end

% step 3: calculate the ground distance
drange = prange/curPd_range;
dvr = pvr/curPd_range;
daz = paz + curPd_az;

pd_range = curPd_range;
pd_az = curPd_az; 
end