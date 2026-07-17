function trackList = Fun_UpdateTrackByAsscResult_Oracle(trackList, pointList, TPmatch_result, params, frame_id)
    for r = 1:size(TPmatch_result, 1)
        ti = TPmatch_result(r, 1);
        pi = TPmatch_result(r, 2);
        if ti < 1 || ti > length(trackList)
            continue;
        end
        trk = trackList{ti};
        if ~isfield(trk, 'nis_history') || isempty(trk.nis_history)
            trk.nis_history = [];
        end
        if ~isfield(trk, 'asscPointList') || isempty(trk.asscPointList)
            trk.asscPointList = {};
        end
        if pi > 0
            dp = pointList(pi);
            innov = [dp.drange - trk.z_pred(1); wrap_angle_oracle(dp.daz - trk.z_pred(2)); dp.radial_vel_meas - trk.z_pred(3)];
            [~, ~, trk.ukf] = ukf_dispatch('update', trk.ukf, innov);
            trk.assoc_det = dp;
            trk.asscPointList{end+1} = dp;
            trk.nis_history(end+1) = safe_nis(innov, trk.P_zz);
            trk = fun_track_quality_management_and_info_completion_oracle(trk, dp, params, params, frame_id);
        else
            [~, ~, trk.ukf] = ukf_dispatch('update', trk.ukf, []);
            trk.assoc_det = [];
            trk.nis_history(end+1) = NaN;
            trk = fun_track_quality_management_and_info_completion_oracle(trk, [], params, params, frame_id);
        end
        if isfield(params, 'fuzzy_window_size') && length(trk.nis_history) > params.fuzzy_window_size
            trk.nis_history = trk.nis_history(end-params.fuzzy_window_size+1:end);
        end
        trackList{ti} = trk;
    end
end

function a = wrap_angle_oracle(a)
    while a > 180
        a = a - 360;
    end
    while a < -180
        a = a + 360;
    end
end

function nis = safe_nis(innov, P_zz)
    try
        nis = innov' * (P_zz \ innov);
    catch
        nis = NaN;
    end
end
