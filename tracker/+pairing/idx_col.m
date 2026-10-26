function c = idx_col(I, id)
% idx_col — 把航迹 ID 映射为索引矩阵列号（0 表示不在索引内）
    c = 0;
    if isempty(I.id2col) || id < 1 || id > numel(I.id2col), return; end
    c = I.id2col(id);
end
