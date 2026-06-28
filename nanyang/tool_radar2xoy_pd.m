function [pos_x, pos_y] = tool_radar2xoy_pd(range, az, sysPara)
modeindex = sysPara.ucMode;
if modeindex <= 4
    [pos_x, pos_y] = tool_radar2xoy_real_pd(range, az, sysPara);
else
    [pos_x, pos_y] = tool_radar2xoy_estimate_pd(range, az, sysPara);
end
end

function [pos_x, pos_y] = tool_radar2xoy_real_pd(range, az, sysPara)
tx = sysPara.tx_BLH;
rx = sysPara.rx_BLH;
[baseline, tx_az] = distance(rx(1), rx(2), tx(1), tx(2), referenceEllipsoid('WGS84'));
baseline = baseline/1e3; % in km

phi = (az - tx_az); 

r1 = 1/2*(range^2 - baseline^2)/(range - baseline*cosd(phi));

pos_x = r1 * sind(az);
pos_y = r1 * cosd(az);

end

function [pos_x, pos_y] = tool_radar2xoy_estimate_pd(range, az, sysPara)

if nargin == 3
    h0 = 200; %km for summer
    % h0 = 300; % km for winter
end

rx = sysPara.rx_BLH;
tx = sysPara.tx_BLH;
[baseline, tx_az] = distance(rx(1), rx(2), tx(1), tx(2), referenceEllipsoid('WGS84'));
% R_earth = 6371.13; % km
% baseline = arclen/180*pi;
baseline = baseline/1e3; % in km

% estimate of pd
sin_theta = h0/(range/4);
if abs(sin_theta) > 1
    pos_x = 0; pos_y = 0;
    return;
end
cos_theta = sqrt(1-sin_theta^2);
pd = cos_theta;
rho_sum = pd * range; 

rho1 = 0.5 * (rho_sum^2 - baseline^2)/(rho_sum - baseline*cosd(tx_az-az));
% rho1 = 0.5 * (rho_sum^2 - baseline^2)/(rho_sum + baseline*sind( az));
pos_x = rho1 * sind(az);
pos_y = rho1 * cosd(az);
end