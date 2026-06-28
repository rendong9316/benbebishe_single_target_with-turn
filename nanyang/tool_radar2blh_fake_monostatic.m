%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% get the target latitude and longitude from measured round-trip distance 
% and azimuth (north to east)
% Input:
% tx_BLH
% rx_BLH
% drange: round-trip distance
% dtheta: azimuth (north to east)
% Output
% tgtBLH
% ---------------------------------------------------------------------
% Author: Jun Geng
% Date: 2025-12-12
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function tgt_BLH = tool_radar2blh_fake_monostatic(tx_BLH, rx_BLH, drange, dtheta)

run('tool_header.m'); 

radar_BLH = (tx_BLH+rx_BLH)/2; 
radar_lat = radar_BLH(1);
radar_lon = radar_BLH(2);
arcLen = drange/2; 
az = dtheta; 

 [lat_out, lon_out] = reckon(radar_lat, radar_lon, arcLen, az, R_earth); 
 
 tgt_BLH =  [lat_out.', lon_out.'] ; 
end