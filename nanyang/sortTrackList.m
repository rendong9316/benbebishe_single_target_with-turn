%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% sort the tracks in the track list by their type: type 1, type 2, type 3
% and type 4
%
% Input:
% trackList: the unsorted track list 
% 
% Output:
% trackList: the sorted track list
% ----------------------------------------------------------------------
% Date: 2022-03-31
% Author : Jun @ HIT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function sortedTrackList = sortTracList(trackList)
% 
% % initialization
% % containers
% track_type1_list = [];
% track_type2_list = [];
% track_type3_list = [];
% track_type4_list = [];
% % counters
% type1_cnt = 0;
% type2_cnt = 0;
% type3_cnt = 0;
% type4_cnt = 0;
% 
% % sort the track List
% trackNum = length(trackList);
% for tt = 1:trackNum
%     curTrack = trackList(tt); % scan over the list
%     if curTrack.Type == 1
%         type1_cnt = type1_cnt + 1;
%         track_type1_list = [track_type1_list, curTrack];
%     elseif curTrack.Type == 2
%         type2_cnt = type2_cnt + 1;
%         track_type2_list = [track_type2_list, curTrack];
%     elseif curTrack.Type == 3
%         type3_cnt = type3_cnt + 1;
%         track_type3_list = [track_type3_list, curTrack];
%     elseif curTrack.Type == 4
%         type4_cnt = type4_cnt + 1;
%         track_type4_list = [track_type4_list, curTrack];
%     end
% end
% 
% % sort the list
% sortedTrackList = [track_type1_list, track_type2_list, ...
%     track_type3_list, track_type4_list];
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% sort track according to the type
% use sort function to increase the speed of excution 
% ----------------------------------------------------------------------
% Modified: 2022-04-23
% Author : Jun @ HIT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function sortedTrackList = sortTracList(trackList)
% 
% tracks_type = [trackList(:).Type];
% 
% [~, ind] = sort(tracks_type);
% 
% sortedTrackList = trackList(ind);
% 
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% sort track according to the type
% use sort function to increase the speed of excution 
% ----------------------------------------------------------------------
% Modified: 2022-04-23
% Author : Jun @ HIT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function sortedTrackList = sortTrackList(trackList)

if isempty(trackList)
    sortedTrackList = [];
    return;
end
% sort according to the following rule
% 1. reliable and maintained track first, then the starting track, then the
% lost track
% 2. for the reliable and maintained tracks, sort according to their score,
% score = track length + track quality

% step 1: sort tracks according to their type 
tracks_type = [trackList(:).Type];
[~, ind] = sort(tracks_type, 'ascend');
sortedTrackList = trackList(ind);

% step2: selected the good tracks
tracks_type = [sortedTrackList(:).Type];
good_ind = find(tracks_type > 6); % type 1, 2, 3 and 4
good_tracks = sortedTrackList(good_ind);
tracks_length = [good_tracks(:).AsscPointCnt];
tracks_quality = [good_tracks(:).Quality];

% step 3: calculate track scores
% tracks_score = tracks_length;
% tracks_score = tracks_quality;
tracks_score = tracks_length + tracks_quality;
% step 4: re-sort according to score
[~, ind] = sort(tracks_score, 'descend');
sorted_good_tracks = good_tracks(ind);

% step 5: put the sorted result back
sortedTrackList(good_ind) = sorted_good_tracks;

end