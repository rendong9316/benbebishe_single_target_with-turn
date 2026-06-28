%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Decription
% Input
% candidate_track: a group of possible associate point
% Output:
% isValid: 1 if candidate_track is a good track, 0 otherwise
% -----------------------------------------------------------
% Author: Jun Geng @ 2025-09-08
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function isValid = fun_check_track_validation(candidate_track)

% initialze the output
header; 
DEBUG = 0; 

% 1. determine the track length

% 2. make judgement on the intial part of the each track, which if the
% initial part is unqualified, then drop the whole track; 
asscPointList = candidate_track.asscPointList; 
asscTime = [asscPointList(:).time]; % input x 
asscFrame = [asscPointList(:).frameID]; % input x
asscRange = [asscPointList(:).prange]; % input y
asscVr = [asscPointList(:).pvr]; % input y
asscAz = [asscPointList(:).paz]; % input y
trackLen = length(asscTime);

% (1) drop the track with random range
% 【天波适配】群距离受电离层几何放大，Vr预测精度有限，放宽门限
delta_R = 200; %km (原45→200)
prdctR = zeros(1, trackLen); 
prdctR(1) = asscRange(1);
for ff = 2:trackLen
    deltaT = (asscTime(ff) - asscTime(ff-1)) * 3600 * 24; % in sec
    prdctR(ff) = prdctR(ff-1) - asscVr(ff) * deltaT/1000; 
end
diff_R = asscRange - prdctR; 
mean_R = mean(diff_R); 
mse_R = sqrt((1/(trackLen-1)) * sum((diff_R - mean_R).^2));
failure_R1 = (mse_R > delta_R); 

% (2) drop the track if the trend of range is different from velocity
[r0, VK] = robustMinSquareErr((asscTime-asscTime(1))*3600*24, asscRange); % slope
% failure_R2 = ((VK * median(asscVr)) > 0);
failure_R2 = 0; 

% (3) drop the track with increasing velocity
% mean_vr = mean(asscVr);
% [v0, accV] = robustMinSquareErr((asscTime-asscTime(1))*3600*24, asscVr); % slope
% if abs(mean_vr) < 0 %MIN_RADIAL_VELOCITY
%     % for normal target, check the velocity trend
%     accTh = 0.002; % acceration, m/s^2 
%     failure_V2 = (accV > accTh); 
% else
%     % for radial track, check the velocity error
%     accV = min(accV, 0); 
%     failure_V2 = 0;
% end

% accTh = 0.002; % acceration, m/s^2 
[v0, accV] = robustMinSquareErr((asscTime-asscTime(1))*3600*24, asscVr); % slope
% failure_V2 = (accV > accTh); 
failure_V2 = 0; 

% (3) drop the track with random velocity
delta_V = 200; %m/s (原4→200, 天波Vr变化大)
prdctV = v0 + accV * (asscTime-asscTime(1))*3600*24; 
diff_V = asscVr - prdctV; 
mean_V = mean(diff_V); 
mse_V = sqrt((1/trackLen) * sum((diff_V - mean_V).^2));
failure_V1 = (mse_V > delta_V); 

% newly added a rule to rule out the bad azimuth 
% (5) drop the track with random azimuth
delta_A = 7.5; % deg
mean_A = mean(asscAz); 
ind = find(abs(asscAz - mean_A) > delta_A); 
failure_A = (~isempty(ind));

isValid = ~(failure_R1 | failure_R2 | failure_V1 | failure_V2 | failure_A);

% -----------------------------------------------------------------------
% the following program is used to detect which rule blocks most of tracks
% persistent failure_R1_cnt; 
% if isempty(failure_R1_cnt)
%     failure_R1_cnt = 0;
% end
% persistent failure_R2_cnt;
% if isempty(failure_R2_cnt)
%     failure_R2_cnt = 0;
% end
% persistent failure_V2_cnt;
% if isempty(failure_V2_cnt)
%     failure_V2_cnt = 0;
% end
% persistent failure_V1_cnt; 
% if isempty(failure_V1_cnt)
%     failure_V1_cnt = 0;
% end
% persistent failure_A_cnt;
% if isempty(failure_A_cnt)
%     failure_A_cnt = 0;
% end 
% if ~isValid
%     if failure_R1
%         failure_R1_cnt = failure_R1_cnt + 1;
%     end
%     
%     if failure_R2
%         failure_R2_cnt = failure_R2_cnt + 1;
%     end
%     
%     if failure_V1
%         failure_V1_cnt = failure_V1_cnt + 1;
%     end
%     
%     if failure_V2
%         failure_V2_cnt = failure_V2_cnt + 1;
%     end
%     
%     if failure_A
%         failure_A_cnt = failure_A_cnt + 1;
%     end
%     disp([failure_R1_cnt, failure_R2_cnt, failure_V1_cnt, failure_V2_cnt, failure_A_cnt]);
% end
% -----------------------------------------------------------------------

% debug to display the detail information
if DEBUG == 1
    time_diff = (asscTime-asscTime(1))*3600*24; 
    figure(1);
    subplot(1, 2, 1)
    plot(asscFrame, asscRange, 'r*-'); hold on; plot(asscFrame, prdctR, 'b-o'); 
    xlabel('frameID'); ylabel('range (km)'); 
    title([' MSE: ', num2str(mse_R), ' failureR=' num2str(failure_R1|failure_R2)]);
    hold off; 
    subplot(1, 2, 2)
    plot(asscFrame, asscVr, 'r*-'); hold on; plot(asscFrame, accV*time_diff + v0, 'b-o'); 
    xlabel('frameID'); ylabel('velocity (m/s)'); 
    title([' MSE: ', num2str(mse_V), 'failureV=' num2str(failure_V1|failure_V2)]);
    % title(['distance err:', num2str(diff_R), 'MSE: ', num2str(mse_R)]);
    hold off
end

end