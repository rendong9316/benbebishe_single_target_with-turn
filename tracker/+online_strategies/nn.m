function [cands, S] = strategy_nn(S, local1, local2, k, params)
%STRATEGY_NN 朴素最近邻基线
%
%   只用位置距离，不设任何门限、不做时间连续性 / 马氏 / 速度校验。
%   对应开题所述"将一条航迹与另一传感器中距离最近的航迹关联"。

    cands = online_assoc.empty_cands();
    for i = 1:numel(local1)
        for j = 1:numel(local2)
            ev = online_assoc.evidence(local1(i), local2(j));
            if ~isfinite(ev.d2), continue; end
            sc = 1 / (1 + ev.dist_km);
            cands(end+1) = online_assoc.make_cand(local1(i).id, local2(j).id, sc, ev, 1); %#ok<AGROW>
        end
    end
end
