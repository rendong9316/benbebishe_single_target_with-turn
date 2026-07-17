function track = fun_track_quality_management_and_info_completion_oracle(track, asscPoint, sysPara, params, frame_id)
    if ~isfield(track, 'TotalPointCnt'), track.TotalPointCnt = 0; end
    if ~isfield(track, 'AsscPointCnt'), track.AsscPointCnt = 0; end
    if ~isfield(track, 'TotalLostPointCnt'), track.TotalLostPointCnt = 0; end
    if ~isfield(track, 'SuccLossPointCnt'), track.SuccLossPointCnt = 0; end
    if ~isfield(track, 'Quality'), track.Quality = get_field_or_default(track, 'quality', params.oracle_confirm_quality); end
    if ~isfield(track, 'life'), track.life = 0; end
    if ~isfield(track, 'death_reason'), track.death_reason = ''; end

    has_assoc = ~isempty(asscPoint);
    track.TotalPointCnt = track.TotalPointCnt + 1;
    track.life = track.life + 1;

    if has_assoc
        track.AsscPointCnt = track.AsscPointCnt + 1;
        track.SuccLossPointCnt = 0;
        track.updateFlag = 1;
        track.Quality = min(track.Quality + 1, params.oracle_max_quality);
    else
        track.TotalLostPointCnt = track.TotalLostPointCnt + 1;
        track.SuccLossPointCnt = track.SuccLossPointCnt + 1;
        track.updateFlag = 0;
        track.Quality = max(track.Quality - params.oracle_loss_quality_penalty, 0);
    end

    old_type = track.Type;
    track.Type = transition_type(track, has_assoc, params);
    if old_type ~= params.HISTORY_TRACK && track.Type == params.HISTORY_TRACK
        if isempty(track.death_frame)
            track.death_frame = frame_id;
        end
        if isempty(track.death_reason)
            track.death_reason = 'k_loss';
        end
    end

    track.type = track.Type;
    track.quality = track.Quality;
    track.missed = track.SuccLossPointCnt;
    if isfield(track, 'ukf') && isfield(track.ukf, 'x') && numel(track.ukf.x) >= 3
        track.lat = track.ukf.x(3);
        track.lon = track.ukf.x(1);
        outPoint = struct('frameID', frame_id, 'lon', track.lon, 'lat', track.lat);
        if ~isfield(track, 'smoothPointList') || isempty(track.smoothPointList)
            track.smoothPointList = {outPoint};
        else
            track.smoothPointList{end+1} = outPoint;
        end
        track.outputPointList = track.smoothPointList;
    end
    track.isNewTrack = 0;
end

function type = transition_type(track, has_assoc, params)
    type = track.Type;
    if type == params.HISTORY_TRACK
        return;
    end
    if track.SuccLossPointCnt >= params.tracker_K_loss
        type = params.HISTORY_TRACK;
        return;
    end
    if type == params.RELIABLE_TRACK && track.Quality < params.oracle_maintain_quality
        type = params.MAINTAIN_TRACK;
    elseif type == params.MAINTAIN_TRACK && has_assoc && track.Quality >= params.oracle_confirm_quality
        type = params.RELIABLE_TRACK;
    elseif type == params.TEMPORARY_TRACK && has_assoc && track.Quality >= params.oracle_confirm_quality
        type = params.RELIABLE_TRACK;
    end
end

function v = get_field_or_default(s, name, default_value)
    if isstruct(s) && isfield(s, name)
        v = s.(name);
    else
        v = default_value;
    end
end
