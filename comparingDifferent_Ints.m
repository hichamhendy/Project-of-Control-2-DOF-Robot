%%
% NMPC -- Theory and Applications
% Course at Karlsruhe Institute of Technology
%  
%     Optimal loop Optimal control
%
%
% Autor:    Hesham Hendy
% Email: Hesham.hendy@student.kit.edu
% Date:     12-10-2019
% Status:   final
%%
clear;
clc;
cons = constants();


[x_opt_30_expEuler, u_opt_30_expEuler, t] = ocp_approach(30, 'explicitEuler');
[x_opt_30_expRK, u_opt_30_expRK, t] = ocp_approach(30, 'explicitRungeKutta');
[x_opt_30_midp, u_opt_30_midp, t] = ocp_approach(30, 'midpointstep');
[x_opt_30_heun, u_opt_30_heun, t] = ocp_approach(30, 'heun');


x_solution_expEuler = x_opt_30_expEuler ;
u_solution_expEuler = u_opt_30_expEuler ; 

x_solution_expRK  = x_opt_30_expRK ;
u_solution_expRK  = u_opt_30_expRK ; 

x_solution_midp = x_opt_30_midp ;
u_solution_midp = u_opt_30_midp ; 

x_solution_heun = x_opt_30_heun ;
u_solution_heun = u_opt_30_heun ; 

%% plot q1-q2-plane
figure;
fig1 = gcf;
fig1.PaperUnits = 'inches';
fig1.PaperPosition = [0, 0, 8, 8];

subplot(2, 2, 1);
axis([0, 3, -5.5, 2]);
hold on;
grid on;
plot(t, x_solution_expEuler(1, :), 'LineWidth', 2);
plot(t, x_solution_expRK(1, :), 'LineWidth', 2);
plot(t, x_solution_midp(1, :), 'LineWidth', 2);
plot(t, x_solution_heun(1, :), 'LineWidth', 2);
xlabel('sec');
ylabel('rad');
title('Angles on q1 plane');
legend('ExpEuler', 'ExpRKutta', 'Midpoint','Heun','Location','southeast');

subplot(2, 2, 2);
axis([0, 3, -4.5, 1]);
hold on;
grid on;
plot(t, x_solution_expEuler(2, :), 'LineWidth', 2);
plot(t, x_solution_expRK(2, :), 'LineWidth', 2);
plot(t, x_solution_midp(2, :), 'LineWidth', 2);
plot(t, x_solution_heun(2, :), 'LineWidth', 2);
xlabel('sec');
ylabel('rad');
title('Angles on q2 plane');
legend('ExpEuler', 'ExpRKutta', 'Midpoint','Heun','Location','southeast');

subplot(2, 2, 3);
axis([0, 3, -3/2*pi, 3/2*pi]);
hold on;
grid on;
plot(t, x_solution_expEuler(3, :), 'LineWidth', 2);
plot(t, x_solution_expRK(3, :), 'LineWidth', 2);
plot(t, x_solution_midp(3, :), 'LineWidth', 2);
plot(t, x_solution_heun(3, :), 'LineWidth', 2);
xlabel('sec');
ylabel('rad/s');
title('Angualr Velocity on q1 plane');
legend('ExpEuler', 'ExpRKutta', 'Midpoint','Heun','Location','southeast');

subplot(2, 2, 4);
axis([0, 3, -3/2*pi, 3/2*pi]);
hold on;
grid on;
plot(t, x_solution_expEuler(4, :), 'LineWidth', 2);
plot(t, x_solution_expRK(4, :), 'LineWidth', 2);
plot(t, x_solution_midp(4, :), 'LineWidth', 2);
plot(t, x_solution_heun(4, :), 'LineWidth', 2);
xlabel('sec');
ylabel('rad/s');
title('Angualr Velocity on q2 plane');
legend('ExpEuler', 'ExpRKutta', 'Midpoint','Heun','Location','southeast');
%print('different_integrator_q1q2','-dpng','-r0'); 

%% plot x-y-plane
x_expEuler = cons.l1 .* cos(x_solution_expEuler(1, :)) + cons.l2 .* cos(x_solution_expEuler(1, :) + x_solution_expEuler(2, :));
y_expEuler = cons.l1 .* sin(x_solution_expEuler(1, :)) + cons.l2 .* sin(x_solution_expEuler(1, :) + x_solution_expEuler(2, :));

x_expRK = cons.l1 .* cos(x_solution_expRK(1, :)) + cons.l2 .* cos(x_solution_expRK(1, :) + x_solution_expRK(2, :));
y_expRK = cons.l1 .* sin(x_solution_expRK(1, :)) + cons.l2 .* sin(x_solution_expRK(1, :) + x_solution_expRK(2, :));

x_midp = cons.l1 .* cos(x_solution_midp(1, :)) + cons.l2 .* cos(x_solution_midp(1, :) + x_solution_midp(2, :));
y_midp = cons.l1 .* sin(x_solution_midp(1, :)) + cons.l2 .* sin(x_solution_midp(1, :) + x_solution_midp(2, :));

x_heun = cons.l1 .* cos(x_solution_heun(1, :)) + cons.l2 .* cos(x_solution_heun(1, :) + x_solution_heun(2, :));
y_heun = cons.l1 .* sin(x_solution_heun(1, :)) + cons.l2 .* sin(x_solution_heun(1, :) + x_solution_heun(2, :));

figure;
fig2 = gcf;
fig2.PaperUnits = 'inches';
fig2.PaperPosition = [0, 0, 8, 8];

subplot(2, 1, 1);
axis([0, 3, -1, 1]);
hold on;
grid on;
plot(t, x_expEuler, 'LineWidth', 2);
plot(t, x_expRK, 'LineWidth', 2);
plot(t, x_midp, 'LineWidth', 2);
plot(t, x_heun, 'LineWidth', 2);
xlabel('sec');
ylabel('m/sec');
title('Motion on x-plane');
legend('ExpEuler', 'ExpRKutta', 'Midpoint','Heun','Location','southeast');

subplot(2, 1, 2);
axis([0, 3, -0.5, 1]);
hold on;
grid on;
plot(t, y_expEuler, 'LineWidth', 2);
plot(t, y_expRK, 'LineWidth', 2);
plot(t, y_midp, 'LineWidth', 2);
plot(t, x_heun, 'LineWidth', 2);
xlabel('sec');
ylabel('m/sec');
title('Motion on y-plane');
legend('ExpEuler', 'ExpRKutta', 'Midpoint','Heun','Location','southeast');
%print('different_integrator_xy','-dpng','-r0');

%% plot inputs
figure;
fig3 = gcf;
fig3.PaperUnits='inches';
fig3.PaperPosition = [0 0 8 8];

subplot(2, 1, 1);
axis([0, 3, -1000, 1000]);
hold on;
grid on;
stairs(t(2:length(t)), u_solution_expEuler(1, :), 'LineWidth', 2);
stairs(t(2:length(t)), u_solution_expRK(1, :), 'LineWidth', 2);
stairs(t(2:length(t)), u_solution_midp(1, :), 'LineWidth', 2);
stairs(t(2:length(t)), u_solution_heun(1, :), 'LineWidth', 2);
xlabel('sec');
ylabel('N.m');
title('Applied Torque by U1');
legend('ExpEuler', 'ExpRKutta', 'Midpoint','Heun');

subplot(2, 1, 2);
axis([0, 3, -1000, 1000]);
hold on;
grid on;
stairs(t(2:length(t)), u_solution_expEuler(2, :), 'LineWidth', 2);
stairs(t(2:length(t)), u_solution_expRK(2, :), 'LineWidth', 2);
stairs(t(2:length(t)), u_solution_midp(2, :), 'LineWidth', 2);
stairs(t(2:length(t)), u_solution_heun(2, :), 'LineWidth', 2);
xlabel('sec');
ylabel('N.m');
title('Applied Torque by U2');
legend('ExpEuler', 'ExpRKutta', 'Midpoint','Heun');
% print('different_integrator_input','-dpng','-r0');

%% ========================================================================
%  subfunctions
%  =========================================================================

function [x_sol, u_sol, t] = ocp_approach(sample_points,type)
import casadi.*;

opti = casadi.Opti();
const = constants();

%% System limits 
% Notice (for me) compare to direct_single_shooting.m codes, also in CasADi’s examples collection

t_f = 1;
t_0 = 0;
N = sample_points; % % number of control intervals % Time horizon %horzions (optimizied)
timeSteps = (t_f - t_0) / N;

%% optimized Variables
% The control trajectory is parameterized using some piecewise smooth approximation, typically piecewise constant.

x = opti.variable(4, N + 1); % Optimization of 4 states (amount of parameters) on N control intervals + 1 predicted state
u = opti.variable(2, N); % Optimization of 2 inputs controls on N control intervals
tendopt = opti.variable(); % Optimization of the value of the time step ( the optimized final time) 


%% objective function 
S = 1e4;           % weight matrix to act on time
R = 1e02 * eye(2); % weight matrix to act on input torque
Q = 1e01 * eye(4); % weight matrix to act on path

% Notice: Infeasible Problem Detected sometimes !

J = S * tendopt; % responsible for time optimality
t = t_0;

x_0 = const.x_0;
x_end = const.x_end;


for i = 1:N
    % Solving intaial value problem step by step using linear multi step
    % method and appplying constarint that help to stick to the ODE 
    if strcmp(type, 'explicitEuler')
        x_next = explicitEuler(x(:, i), u(:, i), timeSteps, tendopt);
    elseif strcmp(type, 'explicitRungeKutta')
        x_next = explicitRungeKutta(x(:, i), u(:, i),timeSteps, tendopt);
    elseif strcmp(type, 'midpointstep')
        x_next = midpointstep( x(:, i), u(:, i), timeSteps, tendopt);
     elseif strcmp(type, 'heun')
         x_next = heun(x(:, i), u(:, i), timeSteps, tendopt);
    end


    opti.subject_to(x(:,i+1) == x_next);
    t = [t, t(end) + timeSteps];
    J = J + x(:, i)' * Q * x(:, i) + u(:, i)' * R * u(:, i); % Quadratic cost finction with end point
end

opti.minimize(J);

%% constraints formulation

opti.subject_to(x(:, 1) == x_0);
opti.subject_to(x(:, end) == x_end);
opti.subject_to(const.omega_lb <= x(3, :) <= const.omega_ub);
opti.subject_to(const.omega_lb <= x(4, :) <= const.omega_ub);
opti.subject_to(const.u_lb / 1000 <= u(1, :) <=  const.u_ub / 1000);
opti.subject_to(const.u_lb / 1000 <= u(2, :) <= const.u_ub / 1000);

opti.subject_to(0 < tendopt < 3);

%% initial guess
opti.set_initial(x, repmat([0;0;0;0], [1, N + 1]));
opti.set_initial(u, repmat([0;0],[1, N]));

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
t_opt_sol = full(sol.value(tendopt));
t = t_opt_sol .* t; 

end

function x_next = explicitEuler(x, u, h ,t_opt)
%Solving an ODR using one-Step Method
% Computation is performed without iteration
 
x_next = x + h .* stateSpace_ODE(x, u,t_opt); % revise Buchter Diagram

end

function x_next = explicitRungeKutta( x, u, h ,t_opt)
% % Fixed step Runge-Kutta 4th order integrator
k1 = stateSpace_ODE( x, u, t_opt);
k2 = stateSpace_ODE( x + k1 * h / 2, u ,  t_opt + h / 2 );
k3 = stateSpace_ODE( x + k2 * h / 2, u , t_opt + h / 2);
k4 = stateSpace_ODE( x + k3 * h, u , t_opt + h);
% % RK4 steps per interval
x_next = x + (k1 + 2 * k2 + 2 * k3 + k4) * h / 6;
% compare single shooting casadi example    
end

function y = midpointstep(x, u, h ,t_opt)
halfh = h/2;
z = x + halfh * stateSpace_ODE(x, u, t_opt);
y = x + h * stateSpace_ODE(z , u ,t_opt );
end

function xf = heun(x, u, h, t_opt)
 xt = x + h * stateSpace_ODE(x,u,t_opt);
 xf = x + h/2 * (stateSpace_ODE(x,u,t_opt)+stateSpace_ODE(xt,u,t_opt+h));
end