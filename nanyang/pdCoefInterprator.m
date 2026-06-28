%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% convert the 22 pd format into our pd coefficient format
% Input:
% pdCoef: pd coefficient provided by 22
% sysPara: existing system parameter
% isActivate: 0 - this function is inactivated, pd is set to be 1
%             1 - this function is activated, pd is set to be measured
% Output:
% pdInfo: the pd coefficient used in matlab program
% -----------------------------------------------------------------
% Author: Jun @ 2025-12-28
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function pdInfo = pdCoefInterprator(pdCoef, sysPara, isActivate)

%  the 1st sector
if isActivate == 1
    START_LOC = 0; 
    for ii = 1:5
        pdInfo(ii).rIndex = (pdCoef(START_LOC+1) : 100 : pdCoef(START_LOC+2))*2; % start range in km
        pdInfo(ii).az_min = pdCoef(START_LOC+3);
        pdInfo(ii).az_max = pdCoef(START_LOC+4);
    
        % pdcoef for range
        pdInfo(ii).EE1_pdR = pdCoef(START_LOC+5 : START_LOC+15);
        pdInfo(ii).EF2_pdR = pdCoef(START_LOC+16 : START_LOC+26);
        pdInfo(ii).FE3_pdR = pdCoef(START_LOC+27 : START_LOC+37);
        pdInfo(ii).FF4_pdR = pdCoef(START_LOC+38 : START_LOC+48);
    
        % pdcoef for azimuth
        pdInfo(ii).EE1_pdAz = pdCoef(START_LOC+49 : START_LOC+59);
        pdInfo(ii).EF2_pdAz = pdCoef(START_LOC+60 : START_LOC+70);
        pdInfo(ii).FE3_pdAz = pdCoef(START_LOC+71 : START_LOC+81);
        pdInfo(ii).FF4_pdAz = pdCoef(START_LOC+82 : START_LOC+92);

        % next sector
        START_LOC = START_LOC + 92; 
    end
else
    for ii = 1:5
        pd_rIndex = sysPara.rIndex(1) : 100 : sysPara.rIndex(end); 
        pdInfo(ii).rIndex = pd_rIndex; % start range in km
        pdInfo(ii).az_min = sysPara.aIndex(1);
        pdInfo(ii).az_max = sysPara.aIndex(end);
    
        % pdcoef for range
        pdInfo(ii).EE1_pdR = ones(1, length(pd_rIndex)); 
        pdInfo(ii).EF2_pdR = ones(1, length(pd_rIndex));
        pdInfo(ii).FE3_pdR = ones(1, length(pd_rIndex)); 
        pdInfo(ii).FF4_pdR = ones(1, length(pd_rIndex)); 
    
        % pdcoef for azimuth
        pdInfo(ii).EE1_pdAz = zeros(1, length(pd_rIndex)); 
        pdInfo(ii).EF2_pdAz = zeros(1, length(pd_rIndex));
        pdInfo(ii).FE3_pdAz = zeros(1, length(pd_rIndex));
        pdInfo(ii).FF4_pdAz = zeros(1, length(pd_rIndex));
    end
end

end