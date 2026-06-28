function trackList = Fun_PredictNextStep_CV(trackList, sysPara, trackPara)

trackNum = length(trackList);
for tt = 1:trackNum
    curTrack = trackList(tt);
    curTrack = predictNextStep_cv(curTrack, sysPara, trackPara);
    trackList(tt) = curTrack;
end

end