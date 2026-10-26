function ti = truth_idx_of(snaps, id)
% 从快照中反查某航迹 ID 的 truth_idx（真值飞机编号）；缺失返回 NaN
    ti = NaN;
    for k = 1:numel(snaps)
        tl = snaps{k}.trackList;
        if ~iscell(tl), continue; end
        for t = 1:numel(tl)
            if tl{t}.id == id
                if isfield(tl{t}, 'truth_idx'), ti = tl{t}.truth_idx; end
                return;
            end
        end
    end
end
