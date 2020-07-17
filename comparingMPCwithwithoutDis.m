%%
% NMPC -- Theory and Applications
% Course at Karlsruhe Institute of Technology
%  
%     Optimal loop Optimal control
%
%
% Autor:    Hesham Hendy
% Email: Hesham.hendy@student.kit.edu
% Date:     14-10-2019


%%

clear;
clc;
addpath('C:\Program Files\MATLAB/casadi-windows-matlabR2016a-v3.5.0');

[x_mpc, u_mpc, t_mpc] = mpc(0, 0, 0); % nmpc without model error and noise
[x_mpc_disturbed, u_mpc_disturbed, t_mpc_disturbed] = mpc(0, 1, 0.1); % mpc without model error, but with noise, its standard deviation is 0.2

%% plot q1-q2-plane
const = constants();

x_sol_mpc = x_mpc ;
u_sol_mpc = u_mpc  .* [1000; 1000];

x_sol_mpc_disturbed = x_mpc_disturbed ;
u_sol_mpc_disturbed = u_mpc_disturbed  .* [1000; 1000];


figure;
fig1 = gcf;
fig1.PaperUnits = 'inches';
fig1.PaperPosition = [0, 0, 8, 8];

subplot(2, 2, 1);
axis([0, 3, -5.5, 2]);
hold on;
grid on;
plot(t_mpc, x_sol_mpc(1, :),'c', 'LineWidth', 2);
plot(t_mpc_disturbed, x_sol_mpc_disturbed(1, :), 'm', 'LineWidth', 2);
xlabel('sec');
ylabel('rad');
title('Angles on q1 plane');
legend('MPC', 'MPC with Disturbance','Location','southeast');

subplot(2, 2, 2);
axis([0, 3, -4.5, 1]);
hold on;
grid on;
plot(t_mpc, x_sol_mpc(2, :), 'c','LineWidth', 2);
plot(t_mpc_disturbed, x_sol_mpc_disturbed(2, :),'m', 'LineWidth', 2);
xlabel('sec');
ylabel('rad');
title('Angles on q2 plane');
legend('MPC', 'MPC with Disturbance','Location','southeast');

subplot(2, 2, 3);
axis([0, 3, -3/2*pi, 3/2*pi]);
hold on;
grid on;
plot(t_mpc, x_sol_mpc(3, :), 'c','LineWidth', 2);
plot(t_mpc_disturbed, x_sol_mpc_disturbed(3, :), 'm', 'LineWidth', 2);
xlabel('sec');
ylabel('rad/s');
title('Angualr Velocity on q1 plane');
legend('MPC', 'MPC with Disturbance','Location','southeast');

subplot(2, 2, 4);
axis([0, 3, -3/2*pi, 3/2*pi]);
hold on;
grid on;
plot(t_mpc, x_sol_mpc(4, :),'c', 'LineWidth', 2);
plot(t_mpc_disturbed, x_sol_mpc_disturbed(4, :), 'm', 'LineWidth', 2);
xlabel('sec');
ylabel('rad/s');
title('Angualr Velocity on q2 plane');
legend('MPC', 'MPC with Disturbance','Location','southeast');

%print('noise_q1q2','-dpng','-r0');
%% plot x-y-plane

x_heun = const.l1 .* cos(x_sol_mpc(1, :)) + const.l2 .* cos(x_sol_mpc(1, :) + x_sol_mpc(2, :));
y_heun = const.l1 .* sin(x_sol_mpc(1, :)) + const.l2 .* sin(x_sol_mpc(1, :) + x_sol_mpc(2, :));

x_euler = const.l1 .* cos(x_sol_mpc_disturbed(1, :)) + const.l2 .* cos(x_sol_mpc_disturbed(1, :) + x_sol_mpc_disturbed(2, :));
y_euler = const.l1 .* sin(x_sol_mpc_disturbed(1, :)) + const.l2 .* sin(x_sol_mpc_disturbed(1, :) + x_sol_mpc_disturbed(2, :));

figure;
fig2 = gcf;
fig2.PaperUnits = 'inches';
fig2.PaperPosition = [0, 0, 8, 8];

subplot(2, 1, 1);
axis([0, 3, -1, 1]);
hold on;
grid on;
plot(t_mpc, x_heun, 'c','LineWidth', 2);
plot(t_mpc_disturbed, x_euler, 'm','LineWidth', 2);
xlabel('sec');
ylabel('m/sec');
title('Motion on x-plane');
legend('MPC', 'MPC with Disturbance','Location','southeast');

subplot(2, 1, 2);
axis([0, 3, -0.5, 1]);
hold on;
grid on;
plot(t_mpc, y_heun,'c', 'LineWidth', 2);
plot(t_mpc_disturbed, y_euler, 'm','LineWidth', 2);
xlabel('sec');
ylabel('m/sec');
title('Motion on y-plane');
legend('MPC', 'MPC with Disturbance','Location','southeast');


%print('noise_xy','-dpng','-r0');
%% plot inputs
figure;
fig3 = gcf;
fig3.PaperUnits='inches';
fig3.PaperPosition = [0 0 8 8];
subplot(2, 1, 1);

axis([0, 3, -1000, 1200]);
hold on;
grid on;
stairs(t_mpc(2:length(t_mpc)), u_sol_mpc(1, :), 'c', 'LineWidth', 2);
stairs(t_mpc_disturbed(2:length(t_mpc_disturbed)),  u_sol_mpc_disturbed(1, :),'m', 'LineWidth', 2);
xlabel('sec');
ylabel('N.m');
title('Applied Torque by U1');
legend('MPC', 'MPC with Disturbance','Location','southeast');

subplot(2, 1, 2);
axis([0, 3, -1000, 500]);
hold on;
grid on;
stairs(t_mpc(2:length(t_mpc)), u_sol_mpc(2, :), 'c', 'LineWidth', 2);
stairs(t_mpc_disturbed(2:length(t_mpc_disturbed)), u_sol_mpc_disturbed(2, :),'m', 'LineWidth', 2);
xlabel('sec');
ylabel('N.m');
title('Applied Torque by U2');
legend('MPC', 'MPC with Disturbance','Location','southeast');

%print('noise_input','-dpng','-r0');

%% ========================================================================
% subfunctions
% =========================================================================


function [x_mpc, u_mpc, t_mpc] = mpc(model_err, disturbance_err, std_dev)
import casadi.*;
const = constants();

%% OCP-Like
opti = casadi.Opti();

t_horizon = 1;
t_0 = 0;
N = 50;
t_s = (t_horizon - t_0) / N;

t_end = 3;
N_nmpc = t_end / t_s;


x = opti.variable(4, N + 1); %states variable
u = opti.variable(2, N); %control variable

J = 0;
Q = 1*eye(4);
R = 1*eye(2);

x_0 = opti.parameter(4, 1);
x_end = const.x_end;

%Recursive feasibility of the sequence OCP_T_Xf (x(tk)) for all sampling instants tk, k belong to N

for i = 1:N
    if(model_err == 1)
        x_next = explicitEuler_incorrect( x(:, i), u(:, i), t_s * t_end);
    else
        x_next = explicitEuler(  x(:, i), u(:, i), t_s * t_end);
    end
    
    opti.subject_to(x(:, i + 1) == x_next);
    
    J = J + u(:, i)' * R * u(:, i) + (x(:, i + 1) - x_end)' * Q * (x(:, i + 1) - x_end);
end

opti.subject_to(x(:, 1) == x_0);
opti.subject_to(x(:, end) == x_end);
opti.subject_to(const.omega_lb <= x(3, :) <= const.omega_ub);
opti.subject_to(const.omega_lb <= x(4, :) <= const.omega_ub);
opti.subject_to( const.u_lb / 1000 <= u(1, :) <=  const.u_ub / 1000);
opti.subject_to( const.u_lb / 1000 <= u(2, :) <=  const.u_ub / 1000);

opti.subject_to(u(:, end) == 0);

% Decrease of the value function V_T_Xf (x(t)) inbetween sampling two instants tk+1 and tk
opti.minimize(J);
opti.solver('ipopt');

%% MPC

% Parameter setting
% Parameters: Declare any amount of parameters. 
%You must fix them to a specific numerical value before solving, and you may overwrite this value at any time.

x_guess = [];
u_guess = [];
opti.set_value(x_0, const.x_0);

x_ = const.x_0;
x_real = const.x_0;

x_result = [x_];
u_result = [];
t_mpc = [0];

for i = 1:N_nmpc
    if i == 1
        opti.set_initial(x, repmat([0; 0; 0; 0], 1, N + 1)); %repmat: Repeat copies of array
        opti.set_initial(u, repmat([0; 0], 1, N));
    else
        opti.set_initial(x, x_guess);
        opti.set_initial(u, u_guess);
    end
    
    sol = opti.solve();  % Ipopt, a library for large-scale nonlinear optimization.
    
    x_guess = [full(sol.value(x(:, 2:end))), [0; 0; 0; 0]];
    u_guess = [full(sol.value(u(:, 2:end))), [0; 0]];
    
    
    u_ = full(sol.value(u(:, 1)));
    
    if(disturbance_err == 1)
        noise = normrnd(0, std_dev, 4, 1);
        x_real = heun_robot( x_real, u_, t_s * t_end);
        x_ = x_real + noise;

    else
        x_real = heun_robot( x_real, u_, t_s * t_end);
        x_ = x_real;
    end
    % updating the state measurment/estimate of x(tk) by setting it as x_0,
    % as we applied the optimal control u_
    opti.set_value(x_0, x_);
    
    x_result = [x_result, x_real];
    u_result = [u_result, u_];
    t_mpc = [t_mpc, i * t_s];
end

x_mpc = x_result;
u_mpc = u_result;

end


function xf = heun_robot( x, u, h )

     xt = x + h * stateSpace_ODE(x,u,1);
     
     xf = x + h/2 * (stateSpace_ODE(x,u,1)+stateSpace_ODE(xt,u,1));
 
end

function x_next = explicitEuler( x, u, h )

    x_next = x + h .* stateSpace_ODE(x, u, 1);
end

function x_next = explicitEuler_incorrect( x, u, h )

    x_next = x + h .* stateSpace_ODE_incorrect(x, u, 1);
end

function xdot = stateSpace_ODE_incorrect(X_Arg, Steer_Arg, time_instant)
% Reformulation Of the given second-rder diffrential equation
% According to the first requirment, The second order equation should be
% simplified to first order state space system to solve it and get the 
%states directly from it 

const = constants();

 maginified_u = Steer_Arg .* [1000; 1000]; % Symbolic array  elmentwise right division 
 % due to error by solving op and ddebugging it, it's been concluded that u
 % is very small 
    
Bq = [180 + 45  * cos(X_Arg(2)), const.b3 + const.b4 * cos(X_Arg(2)); const.b3 + const.b4 * cos(X_Arg(2)), const.b5];

C = -const.c1 * sin(X_Arg(2)) .* [X_Arg(4), X_Arg(3) + X_Arg(4);-X_Arg(3), 0];

g = [const.g1 * cos(X_Arg(1)) + const.g2 * cos(X_Arg(1) + X_Arg(2));const.g2 * cos(X_Arg(1) + X_Arg(2))];
    
xdot = [X_Arg(3); X_Arg(4); Bq ^ (-1) * (maginified_u - g - C * X_Arg(3:4))] * time_instant;
    
end

