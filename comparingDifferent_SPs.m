%%
% NMPC -- Theory and Applications
% Course at Karlsruhe Institute of Technology
%  
%     Optimal loop Optimal control
%
%
% Autor:    Hesham Hendy
% Email: Hesham.hendy@student.kit.edu
% Date:     13-10-2019
% Status:   final
%%
clear;
clc;
cons = constants();


[x_opt_70_expEuler, u_opt_70_expEuler, t_70] = ocp_approach(70, 'explicitEuler');
[x_opt_100_expEuler, u_opt_100_expEuler, t_100] = ocp_approach(100, 'explicitEuler');
[x_opt_150_expEuler, u_opt_150_expEuler, t_150] = ocp_approach(150, 'explicitEuler');
[x_opt_200_expEuler, u_opt_200_expEuler, t_200] = ocp_approach(200, 'explicitEuler');


x_solution_70_expEuler = x_opt_70_expEuler ;
u_solution_70_expEuler = u_opt_70_expEuler  ; 

x_solution_100_expEuler = x_opt_100_expEuler ;
u_solution_100_expEuler = u_opt_100_expEuler ;

x_solution_150_expEuler = x_opt_150_expEuler ;
u_solution_150_expEuler = u_opt_150_expEuler ;

x_solution_200_expEuler = x_opt_200_expEuler ;
u_solution_200_expEuler = u_opt_200_expEuler ;



%% plot q1-q2-plane
figure;
fig1 = gcf;
fig1.PaperUnits = 'inches';
fig1.PaperPosition = [0, 0, 8, 8];

subplot(2, 2, 1);
axis([0, 3, -5.5, 2]);
hold on;
grid on;
plot(t_70, x_solution_70_expEuler(1, :), 'LineWidth', 2);
plot(t_100, x_solution_100_expEuler(1, :), 'LineWidth', 2);
plot(t_150, x_solution_150_expEuler(1, :), 'LineWidth', 2);
plot(t_200, x_solution_200_expEuler(1, :), 'LineWidth', 2);
xlabel('sec');
ylabel('rad');
title('Angles on q1 plane');
legend('SP=70', 'SP=100', 'SP=150','SP=200','Location','southeast');

subplot(2, 2, 2);
axis([0, 3, -4.5, 1]);
hold on;
grid on;
plot(t_70, x_solution_70_expEuler(2, :), 'LineWidth', 2);
plot(t_100, x_solution_100_expEuler(2, :), 'LineWidth', 2);
plot(t_150, x_solution_150_expEuler(2, :), 'LineWidth', 2);
plot(t_200, x_solution_200_expEuler(2, :), 'LineWidth', 2);
xlabel('sec');
ylabel('rad');
title('Angles on q2 plane');
legend('SP=70', 'SP=100', 'SP=150','SP=200','Location','southeast');

subplot(2, 2, 3);
axis([0, 3, -3/2*pi, 3/2*pi]);
hold on;
grid on;
plot(t_70, x_solution_70_expEuler(3, :), 'LineWidth', 2);
plot(t_100, x_solution_100_expEuler(3, :), 'LineWidth', 2);
plot(t_150, x_solution_150_expEuler(3, :), 'LineWidth', 2);
plot(t_200, x_solution_200_expEuler(3, :), 'LineWidth', 2);
xlabel('sec');
ylabel('rad/s');
title('Angualr Velocity on q1 plane');
legend('SP=70', 'SP=100', 'SP=150','SP=200','Location','southeast');

subplot(2, 2, 4);
axis([0, 3, -3/2*pi, 3/2*pi]);
hold on;
grid on;
plot(t_70, x_solution_70_expEuler(4, :), 'LineWidth', 2);
plot(t_100, x_solution_100_expEuler(4, :), 'LineWidth', 2);
plot(t_150, x_solution_150_expEuler(4, :), 'LineWidth', 2);
plot(t_200, x_solution_200_expEuler(4, :), 'LineWidth', 2);
xlabel('sec');
ylabel('rad/s');
title('Angualr Velocity on q2 plane');
legend('SP=70', 'SP=100', 'SP=150','SP=200','Location','southeast');
%print('different_integrator_q1q2','-dpng','-r0'); 

%% plot x-y-plane
x_70 = cons.l1 .* cos(x_solution_70_expEuler(1, :)) + cons.l2 .* cos(x_solution_70_expEuler(1, :) + x_solution_70_expEuler(2, :));
y_70 = cons.l1 .* sin(x_solution_70_expEuler(1, :)) + cons.l2 .* sin(x_solution_70_expEuler(1, :) + x_solution_70_expEuler(2, :));

x_100 = cons.l1 .* cos(x_solution_100_expEuler(1, :)) + cons.l2 .* cos(x_solution_100_expEuler(1, :) + x_solution_100_expEuler(2, :));
y_100 = cons.l1 .* sin(x_solution_100_expEuler(1, :)) + cons.l2 .* sin(x_solution_100_expEuler(1, :) + x_solution_100_expEuler(2, :));

x_150 = cons.l1 .* cos(x_solution_150_expEuler(1, :)) + cons.l2 .* cos(x_solution_150_expEuler(1, :) + x_solution_150_expEuler(2, :));
y_150 = cons.l1 .* sin(x_solution_150_expEuler(1, :)) + cons.l2 .* sin(x_solution_150_expEuler(1, :) + x_solution_150_expEuler(2, :));

x_200 = cons.l1 .* cos(x_solution_200_expEuler(1, :)) + cons.l2 .* cos(x_solution_200_expEuler(1, :) + x_solution_200_expEuler(2, :));
y_200 = cons.l1 .* sin(x_solution_200_expEuler(1, :)) + cons.l2 .* sin(x_solution_200_expEuler(1, :) + x_solution_200_expEuler(2, :));

figure;
fig2 = gcf;
fig2.PaperUnits = 'inches';
fig2.PaperPosition = [0, 0, 8, 8];

subplot(2, 1, 1);
axis([0, 3, -1, 1]);
hold on;
grid on;
plot(t_70, x_70, 'LineWidth', 2);
plot(t_100, x_100, 'LineWidth', 2);
plot(t_150, x_150, 'LineWidth', 2);
plot(t_200, x_200, 'LineWidth', 2);
xlabel('sec');
ylabel('m/sec');
title('Motion on x-plane');
legend('SP=70', 'SP=100', 'SP=150','SP=200','Location','southeast');

subplot(2, 1, 2);
axis([0, 3, -0.5, 1]);
hold on;
grid on;
plot(t_70, y_70, 'LineWidth', 2);
plot(t_100, y_100, 'LineWidth', 2);
plot(t_150, y_150, 'LineWidth', 2);
plot(t_200, y_200, 'LineWidth', 2);
xlabel('sec');
ylabel('m/sec');
title('Motion on y-plane');
legend('SP=70', 'SP=100', 'SP=150','SP=200','Location','southeast');
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
stairs(t_70(2:length(t_70)), u_solution_70_expEuler(1, :), 'LineWidth', 2);
stairs(t_100(2:length(t_100)), u_solution_100_expEuler(1, :), 'LineWidth', 2);
stairs(t_150(2:length(t_150)), u_solution_150_expEuler(1, :), 'LineWidth', 2);
stairs(t_200(2:length(t_200)), u_solution_200_expEuler(1, :), 'LineWidth', 2);
xlabel('sec');
ylabel('N.m');
title('Applied Torque by U1');
legend('SP=70', 'SP=100', 'SP=150','SP=200');

subplot(2, 1, 2);
axis([0, 3, -1000, 1000]);
hold on;
grid on;
stairs(t_70(2:length(t_70)), u_solution_70_expEuler(2, :), 'LineWidth', 2);
stairs(t_100(2:length(t_100)), u_solution_100_expEuler(2, :), 'LineWidth', 2);
stairs(t_150(2:length(t_150)), u_solution_150_expEuler(2, :), 'LineWidth', 2);
stairs(t_200(2:length(t_200)), u_solution_200_expEuler(2, :), 'LineWidth', 2);

xlabel('sec');
ylabel('N.m');
title('Applied Torque by U2');
legend('SP=70', 'SP=100', 'SP=150','SP=200');
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
J=0;
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
k2 = stateSpace_ODE( x + k1 * h / 2, u ,  t_opt  );
k3 = stateSpace_ODE( x + k2 * h / 2, u , t_opt );
k4 = stateSpace_ODE( x + k3 * h, u , t_opt );
% % RK4 steps per interval
x_next = x + (k1 + 2 * k2 + 2 * k3 + k4) * h / 6;
% compare single shooting casadi example    
end

function y = midpointstep(x, u, h ,t_opt)
halfh = h/2;
z = x + halfh * stateSpace_ODE(x, u, t_opt);
y = x + h * stateSpace_ODE(z , u ,t_opt + halfh);
end

function xf = heun(x, u, h, t_opt)
 xt = x + h * stateSpace_ODE(x,u,t_opt);
 xf = x + h/2 * (stateSpace_ODE(x,u,t_opt)+stateSpace_ODE(xt,u,t_opt));
end