function [smooth_range, smooth_vr, smooth_az, smooth_vx, smooth_vy] = fun_trackfilter_AlphaBeta(curTrack, asscPoint, sysPara)

smooth_range = track_smooth_range(curTrack, asscPoint, sysPara); 
[smooth_vr, smooth_vx, smooth_vy, smooth_az] = track_smooth_velocity_azimuth(curTrack, asscPoint, sysPara); 

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: range smooth
% 
% Input:
% curTrack: current Track, with filled predict information
% asscPoint: associated point. if there has no association, put is as emety
%
% Output:
% smooth_range: smoothed range
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function smooth_range = track_smooth_range(curTrack, asscPoint, sysPara)

winLen = 10; 
if isempty(asscPoint)
    % if you don't have an associated point
    if curTrack.AsscPointCnt < winLen
        % if the track length is small, then the previous smooth value to
        % predict the next value
        % prev_range = curTrack.smoothPointList(curTrack.TotalPointCnt-1).prange;
        % prev_vr = curTrack.smoothPointList(curTrack.TotalPointCnt-1).pvr;
        % true_vr = prev_vr; 
        % smooth_range = prev_range - true_vr * sysPara.T_inter/1e3;
        
        prct_range = curTrack.predictRes(curTrack.TotalPointCnt).prange;
        smooth_range = prct_range; 
    else
        % if the track length is large, then use the interpolation method to
        % predict the next value

        % prev_range = curTrack.smoothPointList(curTrack.TotalPointCnt-1).prange;
        % prev_vr = curTrack.smoothPointList(curTrack.TotalPointCnt-1).pvr; 
        % true_vr = prev_vr; 
        % smooth_range = prev_range - true_vr * sysPara.T_inter/1e3;
        
        prct_range = curTrack.predictRes(curTrack.TotalPointCnt).prange;
        smooth_range = prct_range;
    end
else
    % if you have an associated point

    % take mean value between the associated and the predicted point
    % assc_time = [curTrack.asscPointList(:).time];
    % assc_range = [curTrack.asscPointList(:).range];
    weight = 0.15; % weight could be adjusted, outliers has litte weight

    % assc_range = curTrack.asscPointList(curTrack.AsscPointCnt).range;
    prct_range = curTrack.predictRes(curTrack.TotalPointCnt).prange; 
    assc_range = asscPoint.prange; 
    % prev_range = curTrack.smoothPointList(curTrack.TotalPointCnt-1).prange;
    % prev_vr = curTrack.smoothPointList(curTrack.TotalPointCnt-1).pvr;
    % prct_range = prev_range - prev_vr * sysPara.T_inter/1e3;

    smooth_range = weight * assc_range + (1-weight) * prct_range; 
end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: joint velocity and azimuth smooth
% 
% Input:
% curTrack: current Track, with filled predict information
% asscPoint: associated point. if there has no association, put is as emety
%
% Output:
% smooth_az: smoothed azimuth
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [smooth_vr, smooth_vx, smooth_vy, smooth_az] = track_smooth_velocity_azimuth(curTrack, asscPoint, sysPara);

run('header.m'); 

% constants
DEBUG = 0; 
betaV = 0.7; 

betaAz = 0.9;
betaK = 0.5; 

% get the smoothed value
smoothPointList = [curTrack.smoothPointList(1:end-1)];
smoothTime = [smoothPointList(:).time]; 
smoothFrame = [smoothPointList(:).frameID]; 
smoothRange = [smoothPointList(:).prange];
smoothVr = [smoothPointList(:).pvr];
smoothAz = [smoothPointList(:).paz];
smoothLen = length(smoothTime); 
winLen = 12;
if winLen > smoothLen
    winLen = smoothLen; 
end

% get the measured value
asscPointList = [curTrack.asscPointList(:)]; 
asscTime = [asscPointList(:).time]; % input x 
asscFrame = [asscPointList(:).frameID]; % input x
asscRange = [asscPointList(:).prange]; % input y
asscVr = [asscPointList(:).pvr]; % input y
asscAz = [asscPointList(:).paz]; % input y
asscX = [asscPointList(:).lat]; % assemble variable  % DONG_202512_v1
asscY = [asscPointList(:).lon]; % assemble variable
% asscRr = sqrt(asscX.^2 + asscY.^2);
% asscRt = sqrt((asscX-sysPara.tx_XOY(1)).^2 + (asscY-sysPara.tx_XOY(2)).^2);

% determine if has measurement
if isempty(asscPoint)
    hasMeasurement = 0; 
else
    hasMeasurement = 1;
end

% use different method to smooth
refTime = smoothTime(end);
refVr = smoothVr(end);
refRange = smoothRange(end);
refAz = smoothAz(end); 
curTime = sysPara.datenum;

deltaT = tool_get_time_difference(curTime, refTime, MATLAB_TIME_IN_SEC); % in sec 
% sgn_az = sign(smoothAz(end)-smoothAz(1)); % the evolution direction of azimuth 
[~, temp] = robustMinSquareErr((asscTime(1:end)-asscTime(1))*3600*24, asscAz(1:end)); % slope of azimuth
sgn_az = sign(temp); % the evolution direction of azimuth 
if hasMeasurement
    [~, smoothK] = robustMinSquareErr((smoothTime(end-winLen+1:end)...
        -smoothTime(end-winLen+1))*3600*24,smoothVr(end-winLen+1:end));
    [~, asscK] = robustMinSquareErr((asscTime(max(1, end-5):end)...
        -asscTime(max(1, end-5)))*3600*24, asscVr(max(1, end-5):end)); % slope
    refK = betaK * min(smoothK, 0) + (1-betaK) * min(asscK, 0); 
    V_prdct = refK * deltaT + refVr; 
    dA2 = max(-refK * deltaT^2/refRange/1000, 0); % delta_theta square
    A_prdct = refAz + sgn_az*sqrt(dA2)*180/pi*sign(deltaT);

    smooth_vr = betaV * V_prdct + (1-betaV) * asscPoint.pvr;
    smooth_az = betaAz * A_prdct + (1-betaAz) * asscPoint.paz;
else
    [~, smoothK] = robustMinSquareErr((smoothTime(end-winLen+1:end)...
        -smoothTime(end-winLen+1))*3600*24,smoothVr(end-winLen+1:end));
    refK = min(smoothK, 0); 
    V_prdct = refK * deltaT + refVr; 
    dA2 = max(-refK * deltaT^2/refRange/1000, 0); % delta_theta square
    A_prdct = refAz + sgn_az*sqrt(dA2)*180/pi*sign(deltaT);

    smooth_vr = V_prdct;
    smooth_az = A_prdct;
end

% [pos_x, pos_y] = tool_radar2xoy_bistatic_skywave(sysPara.tx_BLH, sysPara.rx_BLH, outputRange, outputAz);
% [pos_x, pos_y] = tool_radar2xoy_skywave_by_pd(refRange, smooth_az, sysPara); 
% Rr = sqrt(pos_x.^2 + pos_y.^2);
% Rt = sqrt((pos_x-sysPara.tx_XOY(1)).^2 + (pos_y-sysPara.tx_XOY(2)).^2);
% vp1 = -refK * Rr * Rt/(Rr+Rt) * 1000; 
% vp2 = Rr^2/deltaT^2 * dA2 * 1000^2;
% smooth_vx = (smooth_vr/2)^2 + vp1; 
% smooth_vy = (smooth_vr/2)^2 + vp2; 

[smooth_vx, smooth_vy, ~, ~] = estimate_velocity(smoothTime, smoothRange, smoothVr, smoothAz); 

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: estimate_velocity
% 
% Input:
% timeList: a series of time 
% rangeList: a seiries of range, which corresponds to the time List
% vrList: a seiries of vr, which corresponds to the time List
% azList: a seiries of azimuth, which corresponds to the time List
%
% Output:
% vx, vy: velocity on x-axis and y-axis respectively. 
% sog: speed over ground (m/s)
% cog: course over ground (deg)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [vx, vy, sog, cog] = estimate_velocity(timeList, rangeList, vrList, azList, methodID)

if nargin == 4
    methodID = 1;
end

dataLen = length(timeList); 
if dataLen < 2
    vx = 0; 
    vy = 0; 
    sog = 0; 
    cog = 0; 
    warning('at least two points required!');
end

switch(methodID)
    case 1
        [vx, vy, sog, cog] = subfunc_velocityEst_method1(timeList, rangeList, vrList, azList);
    case 2
        [vx, vy, sog, cog] = subfunc_velocityEst_method1(timeList, rangeList, vrList, azList);
    otherwise
        disp('otherwise')
end

end


% -------------------------- sub functions ---------------------
function [vx, vy, sog, cog] = subfunc_velocityEst_method1(timeList, rangeList, vrList, azList)

dataLen = length(timeList);
winLen = 10; 
if dataLen > winLen
    % only select the window length
    timeList = timeList(end-winLen+1 : end);
    rangeList = rangeList(end-winLen+1 : end);
    vrList = vrList(end-winLen+1 : end);
    azList = azList(end-winLen+1 : end);
end
dataLen = length(timeList);

midpoint1 = ceil(dataLen/2);
midpoint2 = floor(dataLen/2)+1; 
% get the left point
time1 = mean(timeList(1:midpoint1)); 
Rr1 = mean(rangeList(1:midpoint1));
vr1 = mean(vrList(1:midpoint1));
Az1 = mean(azList(1:midpoint1));
pos_x1 = Rr1*cosd(90-Az1);
pos_y1 = Rr1*sind(90-Az1); 
% get the right point
time2 = mean(timeList(midpoint2:end)); 
Rr2 = mean(rangeList(midpoint2:end));
vr2 = mean(vrList(midpoint2:end));
Az2 = mean(azList(midpoint2:end));
pos_x2 = Rr2*cosd(90-Az2);
pos_y2 = Rr2*sind(90-Az2);

% get the velocity direction
delta_x = pos_x2 - pos_x1; 
delta_y = pos_y2 - pos_y1; 
cog = atan2d(delta_x, delta_y); % anti-clockwise from x-axis
cog = mod(90-cog, 360); % deg, from north to east

% get the parallel velocity
delta_az = Az2 - Az1; % deg, from north to east, clockwise <-> positive
delta_time = (time2 - time1)*3600*24; 
Rr = (Rr1 + Rr2)/2; 
vp = (Rr*1000) * (delta_az/180*pi) / delta_time; % in m/s

% get the radial velocity
vr = (vr1 + vr2)/4; 

% get the full speed; 
sog = sqrt(vp.^2 + vr.^2); 

% get the vx and vy
vx = sog * cosd(90-cog);
vy = sog * sind(90-cog); 

end


function [vx, vy, sog, cog] = subfunc_velocityEst_method2(timeList, rangeList, vrList, azList)
% [pos_x, pos_y] = tool_radar2xoy_skywave_by_pd(refRange, smooth_az, sysPara); 
[pos_x, pos_y] = tool_radar2xoy_pd(refRange, smooth_az, sysPara); 
Rr = sqrt(pos_x.^2 + pos_y.^2);
Rt = sqrt((pos_x-sysPara.tx_XOY(1)).^2 + (pos_y-sysPara.tx_XOY(2)).^2);
vp1 = -refK * Rr * Rt/(Rr+Rt) * 1000; 
vp2 = Rr^2/deltaT^2 * dA2 * 1000^2;
smooth_vx = (smooth_vr/2)^2 + vp1; 
smooth_vy = (smooth_vr/2)^2 + vp2; 
end
