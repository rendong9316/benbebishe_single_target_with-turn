function newTrack = fun_create_new_track_oracle(det1, det2, ukf_tpl, params, frame_id, next_id, truth_idx, real_hist)
    if nargin < 8
        real_hist = struct('frameID', {}, 'point', {}, 'origIndex', {});
    end

    new_ukf = ukf_dispatch('init', ukf_tpl, det1, det2);
    new_ukf = post_init_multi(new_ukf, params);

    smoothPoint = make_output_point(det2, new_ukf, frame_id);
    asscPointList = cell(1, length(real_hist));
    for i = 1:length(real_hist)
        asscPointList{i} = real_hist(i).point;
    end
    if isempty(asscPointList)
        asscPointList = {det1, det2};
    end

    birth_frame = double(real_hist(1).frameID);
    confirm_frame = frame_id;
    span = confirm_frame - birth_frame + 1;
    assoc_count = length(real_hist);

    newTrack = struct();
    newTrack.id = next_id;
    newTrack.truth_idx = truth_idx;
    newTrack.Type = params.RELIABLE_TRACK;
    newTrack.type = params.RELIABLE_TRACK;
    newTrack.Quality = params.oracle_confirm_quality;
    newTrack.quality = params.oracle_confirm_quality;
    newTrack.isNewTrack = 1;
    newTrack.updateFlag = 1;
    newTrack.TotalPointCnt = span;
    newTrack.AsscPointCnt = assoc_count;
    newTrack.TotalLostPointCnt = span - assoc_count;
    newTrack.SuccLossPointCnt = 0;
    newTrack.missed = 0;
    newTrack.asscPointList = asscPointList;
    newTrack.predictRes = {};
    newTrack.smoothPointList = {smoothPoint};
    newTrack.outputPointList = newTrack.smoothPointList;
    newTrack.ukf = new_ukf;
    newTrack.lat = new_ukf.x(3);
    newTrack.lon = new_ukf.x(1);
    newTrack.life = span;
    newTrack.birth_frame = birth_frame;
    newTrack.confirm_frame = confirm_frame;
    newTrack.death_frame = [];
    newTrack.death_reason = '';
    newTrack.BatchNo = [];
    newTrack.assoc_det = det2;
    newTrack.nis_history = [];
end

function p = make_output_point(det, ukf, frame_id)
    p = det;
    p.frameID = frame_id;
    p.lon = ukf.x(1);
    p.lat = ukf.x(3);
end
