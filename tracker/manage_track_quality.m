% =========================================================================
% manage_track_quality.m
% 航迹质量状态机（适配 Pd=60% 低检出率场景）
% =========================================================================
% 对称计分（±1）+ 降级阈值缓冲:
%   TEMPORARY(6) +关联 → quality+1 → ≥10升级RELIABLE(1)
%   TEMPORARY(6) -关联 → quality-1 → <5降HISTORY(7)
%   RELIABLE(1)  ±关联 → quality±1 → <8降MAINTAIN(2)
%   MAINTAIN(2)  ±关联 → quality±1 → ≥10恢复RELIABLE, <5降HISTORY
%   HISTORY(7)   保持
% =========================================================================

function trackList = manage_track_quality(trackList, active_idx, params, frame_id)
    TYPE_RELIABLE   = 1;
    TYPE_MAINTAIN   = 2;
    TYPE_TEMPORARY  = 6;
    TYPE_HISTORY    = 7;

    for i = 1:length(active_idx)
        t = active_idx(i);
        trk = trackList{t};

        was_associated = ~isempty(trk.assoc_det);

        switch trk.type
            case TYPE_TEMPORARY
                if was_associated
                    trk.quality = min(trk.quality + 1, 15);
                    if trk.quality >= 10
                        trk.type = TYPE_RELIABLE;
                    end
                else
                    trk.quality = trk.quality - 1;
                    if trk.quality < 3
                        trk.type = TYPE_HISTORY;
                        trk.death_frame = frame_id;
                    end
                end

            case TYPE_RELIABLE
                if was_associated
                    trk.quality = min(trk.quality + 1, 15);
                else
                    trk.quality = trk.quality - 1;
                    if trk.quality < 8
                        trk.type = TYPE_MAINTAIN;
                    end
                end

            case TYPE_MAINTAIN
                if was_associated
                    trk.quality = min(trk.quality + 1, 15);
                    if trk.quality >= 10
                        trk.type = TYPE_RELIABLE;
                    end
                else
                    trk.quality = trk.quality - 1;
                    if trk.quality < 3
                        trk.type = TYPE_HISTORY;
                        trk.death_frame = frame_id;
                    end
                end

            case TYPE_HISTORY
                % 保持HISTORY状态不变
        end

        % K_loss仅对TEMPORARY航迹强制终止
        if trk.type == TYPE_TEMPORARY && trk.missed >= params.tracker_K_loss
            trk.type = TYPE_HISTORY;
            trk.death_frame = frame_id;
        end

        trackList{t} = trk;
    end
end
