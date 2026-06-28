%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 辅助函数：检查是否重复航迹
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function is_duplicate = is_duplicate_track(new_indices, existing_indices)

is_duplicate = false;

if isempty(existing_indices)
    return;
end

new_sorted = sort(new_indices);

for i = 1:size(existing_indices, 1)
    existing_sorted = sort(existing_indices(i, :));
    
    if length(new_sorted) == length(existing_sorted) && ...
       all(new_sorted == existing_sorted)
        is_duplicate = true;
        return;
    end
end
end