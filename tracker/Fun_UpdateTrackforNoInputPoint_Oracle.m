function trackList = Fun_UpdateTrackforNoInputPoint_Oracle(trackList, params, frame_id)
    for i = 1:length(trackList)
        trk = trackList{i};
        [~, ~, trk.ukf] = ukf_dispatch('update', trk.ukf, []);
        trk.assoc_det = [];
        if ~isfield(trk, 'nis_history') || isempty(trk.nis_history)
            trk.nis_history = [];
        end
        trk.nis_history(end+1) = NaN;
        trk = fun_track_quality_management_and_info_completion_oracle(trk, [], params, params, frame_id);
        trackList{i} = trk;
    end
end
