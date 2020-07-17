function const = constants()
% Given Parameters 
%   No Need for explanation
    const = struct;
    
    %% definition of system paramters
    const.b1 = 200.0;
    const.b2 = 50.0;
    const.b3 = 23.5;
    const.b4 = 25.0;
    const.b5 = 122.5;
    const.c1 = -25.0;
    const.g1 = 784.8;
    const.g2 = 245.3;
    const.l1 = 0.5;
    const.l2 = 0.5;
    
    %% Plant Limit
    const.u_lb = -1000.0;
    const.u_ub = 1000.0;
    const.omega_lb = -3 * pi / 2;
    const.omega_ub = 3 * pi / 2;
    
    %% Initial Conditions
    const.x_0 = [-5; -4; 0; 0]; % given in task
    const.x_end = [pi / 2; 0; 0; 0]; % given in task
    
end

