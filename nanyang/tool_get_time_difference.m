function timeDiff = tool_get_time_difference(starTime, endTime, tMode)

run('tool_header.m'); 

if tMode == MATLAB_TIME_IN_SEC
    timeDiff = (starTime - endTime) * 3600 * 24; 
elseif tMode == MATLAB_TIME_IN_MIN
    timeDiff = (starTime - endTime) * 60 * 24;
end

end