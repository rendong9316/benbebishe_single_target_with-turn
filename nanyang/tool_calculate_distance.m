function ddist = tool_calculate_distance(curLat, curLon, nextLat, nextLon, unit)

run('tool_header.m'); 

if nargin == 4
    unit = 1; % 1 stands for km, 2 stands for m 
end

ddist = distance(curLat, curLon, nextLat, nextLon, R_earth); 

if 1 == unit
    return; 
elseif 2 == unit
    ddist = ddist * 1000; 
else
    return; 
end


