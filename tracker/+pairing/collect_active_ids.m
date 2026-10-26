function ids = collect_active_ids(snaps)
% 收集快照中所有出现过的活跃航迹 ID（排除 HISTORY type==7 与无效位置）
    ids = [];
    for k = 1:length(snaps)
        if ~isfield(snaps{k}, 'trackList') || isempty(snaps{k}.trackList), continue; end
        trks = snaps{k}.trackList;
        for t = 1:length(trks)
            trk = trks{t};
            if trk.type ~= 7 && ~isnan(trk.lat)
                ids(end+1) = trk.id;  %#ok<AGROW>
            end
        end
    end
    ids = unique(ids);
end
