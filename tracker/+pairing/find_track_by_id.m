function trk = find_track_by_id(snap, tid)
% 在单帧快照的 trackList 中按 ID 查找航迹
    trk = [];
    if ~isfield(snap, 'trackList') || isempty(snap.trackList), return; end
    for t = 1:length(snap.trackList)
        if snap.trackList{t}.id == tid
            trk = snap.trackList{t};
            return;
        end
    end
end
