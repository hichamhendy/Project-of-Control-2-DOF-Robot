function  dxdt = stateSpace_ODE(time_instant, X_Arg, Steer_Arg)
% Reformulation Of the given second-rder diffrential equation
% According to the first requirment, The second order equation should be
% simplified to first order state space system to solve it and get the 
%states directly from it 

const = constants();

 maginified_u = Steer_Arg .* [1000; 1000]; % Symbolic array  elmentwise right division 
 % due to error by solving op and ddebugging it, it's been concluded that u
 % is very small 
    
Bq = [const.b1 + const.b2 * cos(X_Arg(2)), const.b3 + const.b4 * cos(X_Arg(2)); const.b3 + const.b4 * cos(X_Arg(2)), const.b5];

C = -const.c1 * sin(X_Arg(2)) .* [X_Arg(4), X_Arg(3) + X_Arg(4);-X_Arg(3), 0];

g = [const.g1 * cos(X_Arg(1)) + const.g2 * cos(X_Arg(1) + X_Arg(2));const.g2 * cos(X_Arg(1) + X_Arg(2))];
    
dxdt = [X_Arg(3); X_Arg(4); Bq ^ (-1) * (maginified_u - g - C * X_Arg(3:4))] ;
    
end

