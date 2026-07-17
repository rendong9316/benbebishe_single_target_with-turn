function trackList = sortTrackList_oracle(trackList)
    if isempty(trackList)
        return;
    end
    ids = zeros(1, length(trackList));
    for i = 1:length(trackList)
        ids(i) = trackList{i}.id;
    end
    [~, order] = sort(ids);
    trackList = trackList(order);
end
