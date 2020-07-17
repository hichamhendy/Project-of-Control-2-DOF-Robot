function xdot = stateSpace_ODE_incorrect(X_Arg, Steer_Arg, time_instant)
% Reformulation Of the given second-rder diffrential equation with
% considering incorrect parameters of b1 and b1
% According to the first requirment, The second order equation should be
% simplified to first order state space system to solve it and get the 
%states directly from it 

const = constants();

 maginified_u = Steer_Arg .* [1000; 1000]; % Symbolic array  elmentwise right division 
 % due to error by solving op and ddebugging it, it's been concluded that u
 % is very small 
    
Bq = [180 + 45 * cos(X_Arg(2)), const.b3 + const.b4 * cos(X_Arg(2)); const.b3 + const.b4 * cos(X_Arg(2)), const.b5];

C = -const.c1 * sin(X_Arg(2)) .* [X_Arg(4), X_Arg(3) + X_Arg(4);-X_Arg(3), 0];

g = [const.g1 * cos(X_Arg(1)) + const.g2 * cos(X_Arg(1) + X_Arg(2));const.g2 * cos(X_Arg(1) + X_Arg(2))];
    
xdot = [X_Arg(3); X_Arg(4); Bq ^ (-1) * (maginified_u - g - C * X_Arg(3:4))] * time_instant;
    
end

