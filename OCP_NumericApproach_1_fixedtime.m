%%
% NMPC -- Theory and Applications
% Course at Karlsruhe Institute of Technology
%  
%     Optimal loop Optimal control
%
%
% Autor:    Hesham Hendy
% Email: Hesham.hendy@student.kit.edu
% Date:     10-10-2019
% Status:   final

clear;
clc;
addpath('C:\Program Files\MATLAB/casadi-windows-matlabR2016a-v3.5.0');
import casadi.*;

opti = casadi.Opti(); % Optimization problem
const = constants();

%% System limits 
% Notice (for me) compare to direct_single_shooting.m codes, also in CasADi�s examples collection

t_f = 3; % fixed time  
t_0 = 0;
N = 50; % % number of control intervals % Time horizon %horzions (not optimizied)
timeSteps = (t_f - t_0) / N;

%% optimized Variables
% The control trajectory is parameterized using some piecewise smooth approximation, typically piecewise constant.

x = opti.variable(4, N + 1); % Optimization of 4 states (amount of parameters) on N control intervals + 1 predicted state
u = opti.variable(2, N); % Optimization of 2 inputs controls on N control intervals
%%%%tendopt = opti.variable(); % Optimization of the value of the time step ( the optimized final time) 



%% objective function 

% Notice: Infeasible Problem Detected sometimes !

%J = 0;
t = t_0;
% 
% Q = [1 0 0 0;
%     0 1 0 0;
%     0 0 1 0;
%     0 0 0 1];
% 
% R = [1 0;
%     0 1];


x_0 = const.x_0;
x_end = const.x_end;

for i = 1:N
    % continuity constraint
    x_next = explicitEuler(x(:, i), u(:, i), timeSteps,t(end));
    opti.subject_to(x(:,i+1) == x_next);
    t = [t, t(end) + timeSteps];
    % objective function
    %J = J + u(:, i)' * R * u(:, i) + (x(:, i + 1) - x_end)' * Q * (x(:, i + 1) - x_end);
end

opti.minimize(t_f);


%% constraints formulation

opti.subject_to(x(:, 1) == x_0);
opti.subject_to(x(:, end) == x_end);
opti.subject_to(const.omega_lb <= x(3, :) <= const.omega_ub);
opti.subject_to(const.omega_lb <= x(4, :) <= const.omega_ub);
opti.subject_to(const.u_lb / 1000 <= u(1, :) <=  const.u_ub / 1000);
opti.subject_to(const.u_lb / 1000 <= u(2, :) <= const.u_ub / 1000);

%%opti.subject_to(0 < tendopt <= 3);

%% solve
opti.solver('ipopt'); % Ipopt, a library for large-scale nonlinear optimization.

sol = opti.solve();

% Remeber I became an error by running and after placing diffrent break
% points I discovered that the input is tiny so I decided to magnifiy (scale) it by
% solving and unscale it again !

% Run on command window 
% opti.debug.value(x_next)
% opti.debug.value(x,opti.initial())
% opti.debug.show_infeasibilities()

x_sol = full(sol.value(x));
u_sol = full(sol.value(u)) .* [1000; 1000];
%%%%t_opt_sol = full(sol.value(tendopt));

%%t = t_opt_sol .* t;

%% plot in q1-q2-plane
figure;

subplot(2, 2, 1);
hold on;
grid on;
plot(t, x_sol(1, :),'g', 'LineWidth', 2);
xlabel('sec');
ylabel('rad');
title('Angles on q1 plane');
subplot(2, 2, 2);
hold on;
grid on;
plot(t, x_sol(2, :),'g', 'LineWidth', 2);
xlabel('sec');
ylabel('rad');
title('Angles on q2 plane');
subplot(2, 2, 3);
hold on;
grid on;
plot(t, x_sol(3, :), 'g','LineWidth', 2);
xlabel('sec');
ylabel('rad/s');
title('Angualr Velocity on q1 plane');
subplot(2, 2, 4);
hold on;
grid on;
plot(t, x_sol(4, :),'g', 'LineWidth', 2);
xlabel('sec');
ylabel('rad/s');
title('Angualr Velocity on q2 plane');

%% plot in x-y-plane
figure;

x_ = const.l1 .* cos(x_sol(1, :)) + const.l2 .* cos(x_sol(1, :) + x_sol(2, :));
y_ = const.l1 .* sin(x_sol(1, :)) + const.l2 .* sin(x_sol(1, :) + x_sol(2, :));
subplot(2, 1, 1);
hold on;
grid on;
plot(t, x_, 'LineWidth', 2);
xlabel('sec');
ylabel('m/sec');
title('Motion on x-plane');
subplot(2, 1, 2);
hold on;
grid on;
plot(t, y_, 'LineWidth', 2);
xlabel('sec');
ylabel('m/sec');
title('Motion on y-plane');

%% plot inputs
figure;
fig3 = gcf;
fig3.PaperUnits='inches';
fig3.PaperPosition = [0 0 8 8];

subplot(2, 1, 1);
axis([0, 3, -1500, 1500]);
hold on;
grid on;
stairs(t(2:length(t)), u_sol(1, :), 'LineWidth', 2);
xlabel('sec');
ylabel('N.m');
title('Applied Torque by U1');
subplot(2, 1, 2);
axis([0, 3, -1500, 1500]);
hold on;
grid on;
stairs(t(2:length(t)), u_sol(2, :), 'LineWidth', 2);
xlabel('sec');
ylabel('N.m');
title('Applied Torque by U2');


%% ========================================================================
% subfunctions
% =========================================================================

function x_next = explicitEuler(x, u, h,t)
%Solving an ODR using one-Step Method
% Computation is performed without iteration
 
x_next = x + h .* stateSpace_ODE(t, x, u); % revise Buchter Diagram

end