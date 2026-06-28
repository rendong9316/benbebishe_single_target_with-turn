% MID stands for method ID, use different method to calculate gate
function [sigmaR, sigmaA, sigmaV] = get_tracking_gate(fixedR, fixedA, fixedV, floatR, floatA, floatV, lossPtsNum, sysPara, MID)

if MID == 1
    % original method: linear gate
    sigmaR = fixedR + lossPtsNum * sysPara.T_inter * floatR; 
    sigmaA = fixedA + lossPtsNum * sysPara.T_inter * floatA; 
    sigmaV = fixedV + lossPtsNum * sysPara.T_inter * floatV; 
elseif MID == 2
    % linear gate with upper bound
    lossPtsNum = min(lossPtsNum, 4); 
    sigmaR = fixedR + lossPtsNum * sysPara.T_inter * floatR; 
    sigmaA = fixedA + lossPtsNum * sysPara.T_inter * floatA; 
    sigmaV = fixedV + lossPtsNum * sysPara.T_inter * floatV; 
elseif MID == 3
    % raito gate
    p = 0.618; 
    sigmaR = fixedR + lossPtsNum^p * sysPara.T_inter * floatR;
    sigmaA = fixedA + lossPtsNum^p * sysPara.T_inter * floatA;
    sigmaV = fixedV + lossPtsNum^p * sysPara.T_inter * floatV;
elseif MID == 4
    % by table
    lossPtsNum = min(lossPtsNum, 4); 
    coef = [0, 1, 1.8, 2.5, 3.2]; 
    sigmaR = fixedR + coef(lossPtsNum+1) * sysPara.T_inter * floatR;
    sigmaA = fixedA + coef(lossPtsNum+1) * sysPara.T_inter * floatA;
    sigmaV = fixedV + coef(lossPtsNum+1) * sysPara.T_inter * floatV;
else
    sigmaR = fixedR;
    sigmaA = fixedA;
    sigmaV = fixedV;
end