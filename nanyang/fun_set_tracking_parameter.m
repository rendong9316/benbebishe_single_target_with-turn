%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function trackPara = fun_set_tracking_parameter(sysPara)

% for the high rate working mode
trackPara.prdct_r_winLen = 7;
trackPara.prdct_v_winLen = 10;
trackPara.prdct_a_winLen = 11; 

end