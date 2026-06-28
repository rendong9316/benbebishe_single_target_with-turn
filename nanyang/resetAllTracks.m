%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% System Track Reset: Set the status of all tracks to historical tracks and
% clear out the historical point used for track initiation.
% Input:
% trackList: current trackList
% tempTrackList: existing historical points
% Output: 
% trackList: reset trackList
% tempTrackList: empty
% -------------------------------------------------------------------------
% Author: Jun Geng
% Date: 2026-01-24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [trackList, tempTrackList] = resetAllTracks(trackList, tempTrackList)

% run('header.m');
header; 

tempTrackList = []; 

trackNum = length(trackList); 
for tt = 1:trackNum
    curTrack = trackList(tt);
    if curTrack.Type ~= HISTORY_TRACK
        curTrack.Type = HISTORY_TRACK; 
        curTrack.Quality = 3;
        curTrack.updateFlag = 0; 
        trackList(tt) = curTrack;
    end
end

