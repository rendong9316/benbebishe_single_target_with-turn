function trackUpdateInfo = track2reportDataConverter(reportTracks, reportPoints, sysPara) 

% if input is empty
if isempty(reportTracks)
    UPDATE_TRACK = 0;
else
    UPDATE_TRACK = 1;
end

if isempty(reportPoints)
    UPDATE_POINT = 0;
else
    UPDATE_POINT = 1;
end

if (UPDATE_TRACK + UPDATE_POINT) == 0
    % both inputs are empty
    trackUpdateInfo = [];
end

% 1. write head
% headTime = sysPara.datenum;
% time_str = datestr(headTime, 30);
% 
% trackUpdateInfo.unLength = 0; % invalid
% trackUpdateInfo.unInfoType = 0; % invalid
% trackUpdateInfo.unEndian = 0; % invalid
% trackUpdateInfo.unVersion = 0; % invalid
% trackUpdateInfo.unMinor = 0; % invalid
% trackUpdateInfo.unHeadLength = 0; % invalid
% trackUpdateInfo.usYear = str2double(time_str(1:4)); 
% trackUpdateInfo.ucMonth = str2double(time_str(5:6)); 
% trackUpdateInfo.ucDay = str2double(time_str(7:8));
% trackUpdateInfo.ucHour = str2double(time_str(10:11));
% trackUpdateInfo.ucMinute = str2double(time_str(12:13)); 
% trackUpdateInfo.ucSecond = str2double(time_str(14:15)); 
% trackUpdateInfo.ucBack = 0; % invalid
% trackUpdateInfo.ucBack2 = 0; % invalid
% 
% trackUpdateInfo.unPrjNo = 4025; 
% trackUpdateInfo.unFrameNo = sysPara.frameID;
% trackUpdateInfo.unCurBoweiNo = 1; % invalid
% trackUpdateInfo.unIonoSubAreaNo = 1; 
% trackUpdateInfo.unBoweiFaX = 0; % invalid, sysPara.beamDir; 
% trackUpdateInfo.unBoweiWidth = 30;
% trackUpdateInfo.unBoshuCoverMode = 0;
% trackUpdateInfo.unFreUsed = 0;
% trackUpdateInfo.unSearchOrFollow = 0;
% trackUpdateInfo.unFrequency = sysPara.f0 * 1e3; %kHz
% trackUpdateInfo.unStaNumber = 0; % invalid
% trackUpdateInfo.unEndNumber = 0; % invalid
% trackUpdateInfo.unSigBand = 0; %invalid sysPara.Be/1e3; % kHz
% trackUpdateInfo.unSigTimewidth = sysPara.prt; % s
% trackUpdateInfo.unSample = 0; % invalid sysPara.fs; % Hz
% trackUpdateInfo.unAccNum = 0; % invalid sysPara.nAccs;
% trackUpdateInfo.unStaP = 0; % invalid
% trackUpdateInfo.unEndP = 0; % invalid
% trackUpdateInfo.unDistPtNum = 0; % sysPara.RbinNum; % invalid
% trackUpdateInfo.unSynMode = 0;
% trackUpdateInfo.unAlterCyc = sysPara.T_inter; 
% trackUpdateInfo.unFlyNum = 0; % invalid
% trackUpdateInfo.usSigHangJiLen = 0; % invalid
% trackUpdateInfo.ucIP4 = 172; 
% trackUpdateInfo.ucBak4 = [0 0 0 0];

% 2. write data
info_cnt = 0;

if UPDATE_TRACK == 1
    trackNum = length(reportTracks);
    for tt = 1:trackNum
        % get each track
        curTrack = reportTracks(tt);
        trackLen = length(curTrack.smoothPointList(:));
        for ll = 1:trackLen
            info_cnt = info_cnt + 1;
            % get info ready
            trackUpdateInfo(info_cnt).usBatchNo = curTrack.BatchNo; % BatchNo
            startTime = curTrack.smoothPointList(1).time;
            time_str = datestr(startTime, 30);
            trackUpdateInfo(info_cnt).stStarTime = [str2double(time_str(10:11)), str2double(time_str(12:13)), str2double(time_str(14:15))]; % YYYY, MM DD, HH, MM, SS
            trackUpdateInfo(info_cnt).usPtNum = ll; % the number of the points
            trackUpdateInfo(info_cnt).usDist = 0; % inital track no do have travel length - hangcheng km 
            trackUpdateInfo(info_cnt).ucBeamNo = 0; % invalid
            trackUpdateInfo(info_cnt).ucSubAreaNo = 1; % invalid
            trackUpdateInfo(info_cnt).usPDist = round(curTrack.smoothPointList(ll).prange /2*10); % in 0.1 km
            trackUpdateInfo(info_cnt).usPAzi = round(curTrack.smoothPointList(ll).paz * 10); % in 0.1 deg
            trackUpdateInfo(info_cnt).usDDist = round(curTrack.smoothPointList(ll).drange /2*10); % in 0.1 km
            trackUpdateInfo(info_cnt).usDAzi = round(curTrack.smoothPointList(ll).daz *10); % in 0.1 deg
            vx = curTrack.smoothPointList(ll).v_x;
            vy = curTrack.smoothPointList(ll).v_y;
            usTrackAzi = atan2d(vy, vx); % 
            trackUpdateInfo(info_cnt).usTrackAzi = round(mod(90-usTrackAzi, 360) * 10); % in 0.1 deg
            trackUpdateInfo(info_cnt).usTrackSpeed = round(sqrt(vx^2 + vy^2) * 3.6 * 10);  % m/s
            trackUpdateInfo(info_cnt).usRadSpeed = round(curTrack.smoothPointList(ll).dvr * 3.6 * 10); % m/s
            trackUpdateInfo(info_cnt).usDplr = round(curTrack.smoothPointList(ll).pvr/sysPara.lambda * 100); % Hz
            trackUpdateInfo(info_cnt).ucMode = sysPara.ucMode; % 
            trackUpdateInfo(info_cnt).f1PDCoef = curTrack.asscPointList(end).pd_range; % invalid
            trackUpdateInfo(info_cnt).f1AziCoef = curTrack.asscPointList(end).pd_az; % invalid
            trackUpdateInfo(info_cnt).f2PDCoef = 0.8; % invalid
            trackUpdateInfo(info_cnt).f2AziCoef = 0; % invalid
            trackUpdateInfo(info_cnt).f3PDCoef = 0.8; % invalid
            trackUpdateInfo(info_cnt).f3AziCoef = 0; % invalid
            trackUpdateInfo(info_cnt).fLongi = curTrack.smoothPointList(ll).lon; % in deg
            trackUpdateInfo(info_cnt).fLati = curTrack.smoothPointList(ll).lat; % in deg
            % trackUpdateInfo(info_cnt).usDistUnit =  curTrack.asscPointList(ll).Rbin; % RinBin
            % trackUpdateInfo(info_cnt).ucAziUnit = curTrack.asscPointList(ll).Abin; % AzBin
            % trackUpdateInfo(info_cnt).usDplrUnit = curTrack.asscPointList(ll).Dbin; % DBin
            trackUpdateInfo(info_cnt).ucPitchUnit = 1; % invalid
            trackUpdateInfo(info_cnt).ucState = 1; % invalid
            trackUpdateInfo(info_cnt).ucImprtMrk = 0; % invalid
            curTime = curTrack.smoothPointList(ll).time;
            time_str = datestr(curTime, 30);
            try
            trackUpdateInfo(info_cnt).stCurTime = [str2double(time_str(1:4)), str2double(time_str(5:6)), str2double(time_str(7:8))...
                str2double(time_str(10:11)), str2double(time_str(12:13)), str2double(time_str(14:15))];
            catch
                disp(1);
            end
            trackUpdateInfo(info_cnt).usChkThreshold = 11; % 
            if curTrack.smoothPointList(ll).asscFlag == 1
                frameID = curTrack.smoothPointList(ll).frameID;
                asscFrameList = [curTrack.asscPointList(:).frameID];
                ind = find(asscFrameList == frameID); 
                if isempty(ind)
                    trackUpdateInfo(info_cnt).ucSNR = 0;
                else
                    trackUpdateInfo(info_cnt).ucSNR = round(curTrack.asscPointList(ind).snr * 10); % in 0.1dB
                end
            else
                trackUpdateInfo(info_cnt).ucSNR = 0; % BatchNo
            end
%         ucTargetType = 1% BatchNo
%         ucTargetState = 0% BatchNo
%         ucTargetSize = 1% BatchNo
%         unSndDevCode = % BatchNo
%         unRevDevCode = 31605% BatchNo
%         ucAziNo = % BatchNo
%         ucRevDevNo = % BatchNo
%         ucEXTSrcFlg = % BatchNo

%         msg = [usBatchNo, stStarTime, usPtNum, usDist, ucBeamNo, ucSubAreaNo, usPDist, usPAzi, ...
%             usDDist, usDAzi, usTrackAzi, usTrackSpeed, usRadSpeed, usDplr, ucMode, f1PDCoef, ...
%             f1AziCoef, f2PDCoef, f2AziCoef, f3PDCoef, f3AziCoef, fLongi, fLati, usDistUnit, ...
%             ucAziUnit, usDplrUnit, ucPitchUnit, ucState, ucImprtMrk, stCurTime, usChkThreshold, ucSNR]; 
%         msg_str1 = sprintf('%d %d %d %d %d %d %d ', usBatchNo, stStarTime, usPtNum, usDist, ucBeamNo);
%         msg_str2 = sprintf('%d %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d ', ucSubAreaNo, usPDist, usPAzi, usDDist, usDAzi, usTrackAzi, usTrackSpeed, usRadSpeed, usDplr, ucMode);
%         msg_str3 = sprintf('%.2f %.2f %.2f %.2f %.2f %.2f %.6f %.6f %d %d ', f1PDCoef, f1AziCoef, f2PDCoef, f2AziCoef, f3PDCoef, f3AziCoef, fLongi, fLati, usDistUnit, ucAziUnit);
%         msg_str4 = sprintf('%d %d %d %d %d %d %d %d %d %d %.2f %d\n', usDplrUnit, ucPitchUnit, ucState, ucImprtMrk, stCurTime, usChkThreshold, ucSNR);
%         msg = [msg_str1, msg_str2, msg_str3, msg_str4]; 
%         fwrite(fp, msg);
        end
    end
end

if UPDATE_POINT == 1
    ptsNum = length(reportPoints); 
    for pp = 1:ptsNum
        info_cnt = info_cnt + 1;
        curTrack = reportPoints(pp);

        % get info ready
        trackUpdateInfo(info_cnt).usBatchNo = curTrack.BatchNo; % BatchNo
        startTime = curTrack.time;
        time_str = datestr(startTime, 30);
        trackUpdateInfo(info_cnt).stStarTime = [str2double(time_str(10:11)), str2double(time_str(12:13)), str2double(time_str(14:15))]; % YYYY, MM DD, HH, MM, SS
        trackUpdateInfo(info_cnt).usPtNum = curTrack.TotalPointCnt; % invalid
        trackUpdateInfo(info_cnt).usDist = curTrack.travelLen; % in km
        trackUpdateInfo(info_cnt).ucBeamNo = 1; % invalid
        trackUpdateInfo(info_cnt).ucSubAreaNo = 1; % invalid
        trackUpdateInfo(info_cnt).usPDist = round(curTrack.smoothPoint.prange/2 * 10); % in 0.1 km
        trackUpdateInfo(info_cnt).usPAzi = round(curTrack.smoothPoint.paz * 10); % in 0.1 deg
        trackUpdateInfo(info_cnt).usDDist = round(curTrack.smoothPoint.drange/2 * 10); % in 0.1 km
        trackUpdateInfo(info_cnt).usDAzi = round(curTrack.smoothPoint.daz * 10); % in 0.1 deg
        vx = curTrack.smoothPoint.v_x;
        vy = curTrack.smoothPoint.v_y;
        usTrackAzi = atan2d(vy, vx); %
        trackUpdateInfo(info_cnt).usTrackAzi = round(mod(90-usTrackAzi, 360) * 10); % in deg
        trackUpdateInfo(info_cnt).usTrackSpeed = round(sqrt(vx^2 + vy^2) * 10); % m/s
        trackUpdateInfo(info_cnt).usRadSpeed = round(curTrack.smoothPoint.dvr * 3.6 * 10); % m/s
        trackUpdateInfo(info_cnt).usDplr = round(curTrack.smoothPoint.pvr/sysPara.lambda * 100);% BatchNo
        trackUpdateInfo(info_cnt).usTrackAzi = round(mod(90-usTrackAzi, 360) * 10); % in 0.1 deg
        trackUpdateInfo(info_cnt).ucMode = sysPara.ucMode; % invalid
        trackUpdateInfo(info_cnt).f1PDCoef = curTrack.asscPoint.pd_range; % 
        trackUpdateInfo(info_cnt).f1AziCoef = curTrack.asscPoint.pd_az; % invalid
        trackUpdateInfo(info_cnt).f2PDCoef = 0.8; % invalid
        trackUpdateInfo(info_cnt).f2AziCoef = 0; % invalid
        trackUpdateInfo(info_cnt).f3PDCoef = 0.8; % invalid
        trackUpdateInfo(info_cnt).f3AziCoef = 0; % invalid
        trackUpdateInfo(info_cnt).fLongi = curTrack.smoothPoint.lon; % in deg
        trackUpdateInfo(info_cnt).fLati = curTrack.smoothPoint.lat; % in deg
        trackUpdateInfo(info_cnt).usDistUnit = curTrack.asscPoint.Rbin; % RBin;
        trackUpdateInfo(info_cnt).ucAziUnit = curTrack.asscPoint.Abin; % ABin
        trackUpdateInfo(info_cnt).usDplrUnit = curTrack.asscPoint.Dbin; % RinBin;
        trackUpdateInfo(info_cnt).ucPitchUnit = 1; % invalid
        trackUpdateInfo(info_cnt).ucState = 1; % invalid
        trackUpdateInfo(info_cnt).ucImprtMrk = 0; % invalid
        curTime = curTrack.smoothPoint.time;
        time_str = datestr(curTime, 30);
        trackUpdateInfo(info_cnt).stCurTime = [str2double(time_str(1:4)), str2double(time_str(5:6)), str2double(time_str(7:8))...
            str2double(time_str(10:11)), str2double(time_str(12:13)), str2double(time_str(14:15))]; 
        trackUpdateInfo(info_cnt).usChkThreshold = 11; % BatchNo
        if curTrack.smoothPoint.asscFlag == 1
            trackUpdateInfo(info_cnt).ucSNR = round(curTrack.asscPoint.snr * 10); % 0.1dB
        else
            trackUpdateInfo(info_cnt).ucSNR = 0; % BatchNo
        end
    
%         msg_str1 = sprintf('%d %d %d %d %d %d %d ', usBatchNo, stStarTime, usPtNum, usDist, ucBeamNo);
%         msg_str2 = sprintf('%d %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d ', ucSubAreaNo, usPDist, usPAzi, usDDist, usDAzi, usTrackAzi, usTrackSpeed, usRadSpeed, usDplr, ucMode);
%         msg_str3 = sprintf('%.2f %.2f %.2f %.2f %.2f %.2f %.6f %.6f %d %d ', f1PDCoef, f1AziCoef, f2PDCoef, f2AziCoef, f3PDCoef, f3AziCoef, fLongi, fLati, usDistUnit, ucAziUnit);
%         msg_str4 = sprintf('%d %d %d %d %d %d %d %d %d %d %.2f %d\n', usDplrUnit, ucPitchUnit, ucState, ucImprtMrk, stCurTime, usChkThreshold, ucSNR);
%         msg = [msg_str1, msg_str2, msg_str3, msg_str4]; 
%         fwrite(fp, msg); 
    end
end