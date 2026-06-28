function [a0, a1] = robustMinSquareErr(x, y)

% set initial weight
w = ones(size(x)); 

% the first time estimation result
[a0, a1] = weightMinSquareErr(x, y, w);

% the fitting result and the estimate error
hat_y = a1*x + a0; 
err = y - hat_y; 
s = median(abs(err)); 

% reset the weight by the estimation error 
w = min(abs(err/s/6) , 1);
w = (1 - w.^3).^3; 

% the second time estimation result
[a0, a1] = weightMinSquareErr(x, y, w);

% the fitting result and the estimate error
hat_y = a1*x + a0; 
err = y - hat_y; 
s = median(abs(err)); 

% reset the weight by the estimation error 
w = min(abs(err/s/6) , 1);
w = (1 - w.^2).^2; 

% the third time estimation result
[a0, a1] = weightMinSquareErr(x, y, w);

end




function [a0, a1] = weightMinSquareErr(x, y, w)

sum_x2 = sum(x.*x.*w); 
sum_y = sum(y.*w); 
sum_x = sum(x.*w);
sum_xy = sum(x.*y.*w);
sum_w = sum(w);

a0 = (sum_x2*sum_y-sum_x*sum_xy)/(sum_w*sum_x2-sum_x*sum_x);
a1 = (sum_w*sum_xy-sum_x*sum_y)/(sum_w*sum_x2-sum_x*sum_x);

end