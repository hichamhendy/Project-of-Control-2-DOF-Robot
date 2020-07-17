%%
% NMPC -- Theory and Applications
% Course at Karlsruhe Institute of Technology
%  
%     Optimal loop Optimal control
%
%
% Autor:    Hesham Hendy
% Email: Hesham.hendy@student.kit.edu
% Date:     15-10-2019


%%

clear;
clc;
addpath('C:\Program Files\MATLAB/casadi-windows-matlabR2016a-v3.5.0');
const = constants();

% [x_opt_100_expEuler, u_opt_100_expEuler, t] = ocp_approach(30, 'explicitEuler');
[x_opt_40_expRK, u_opt_40_expRK, t] = ocp_approach(40, 'explicitRungeKutta');
% [x_opt_100_midp, u_opt_100_midp, t] = ocp_approach(30, 'midpointstep');
% [x_opt_100_heun, u_opt_100_heun, t] = ocp_approach(30, 'heun');



x_opt_ocp_real = [const.x_0];

for i=1:40
    x = heun(t(i), x_opt_ocp_real(:, i), u_opt_40_expRK(:, i), 0.08);
    %x = explicitRungeKutta(x_opt_ocp_actual(:, i), u_opt_30_expRK(:, i), 0.06);
    x_opt_ocp_real = [x_opt_ocp_real, x];
end


[x_mpc_with_err, u_mpc_with_err, t_mpc_with_err] = mpc(1, 0, 0); % nmpc with model error, but without noise

%% plot q1-q2-plane

x_sol_ocp = x_opt_40_expRK ;
u_sol_ocp = u_opt_40_expRK .* [1000; 1000];

x_sol_ocp_actual = x_opt_ocp_real ;

% % x_solution_nmpc = x_nmpc ./ [p.scal_q1; p.scal_q2; p.scal_w1; p.scal_w2];
% % u_solution_nmpc = u_nmpc ./ [p.scal_u1; p.scal_u2];

x_sol_mpc_with_error = x_mpc_with_err ;
u_sol_mpc_with_error = u_mpc_with_err .* [1000; 1000];


figure;
fig1 = gcf;
fig1.PaperUnits = 'inches';
fig1.PaperPosition = [0, 0, 8, 8];

subplot(2, 2, 1);
axis([0, 3, -5.5, 2]);
hold on;
grid on;
plot(t, x_sol_ocp(1, :), 'LineStyle', '--', 'LineWidth', 2);
plot(t, x_sol_ocp_actual(1, :), 'LineWidth', 2);
plot(t_mpc_with_err, x_sol_mpc_with_error(1, :), 'LineWidth', 2);
xlabel('sec');
ylabel('rad');
title('Angles on q1 plane');
legend('OCP with error', 'OCP Ideal Val', 'NMPC with error','Location','southeast');

subplot(2, 2, 2);
axis([0, 3, -4.5, 1]);
hold on;
grid on;
plot(t, x_sol_ocp(2, :), 'LineStyle', '--', 'LineWidth', 2);
plot(t, x_sol_ocp_actual(2, :), 'LineWidth', 2);
%plot(t_nmpc, x_solution_nmpc(2, :), 'LineWidth', 2);
plot(t_mpc_with_err, x_sol_mpc_with_error(2, :), 'LineWidth', 2);
xlabel('sec');
ylabel('rad');
title('Angles on q2 plane');
legend('OCP with error', 'OCP Ideal Val', 'NMPC with error','Location','southeast');

subplot(2, 2, 3);
axis([0, 3, -3/2*pi, 3/2*pi]);
hold on;
grid on;
plot(t, x_sol_ocp(3, :), 'LineStyle', '--', 'LineWidth', 2);
plot(t, x_sol_ocp_actual(3, :), 'LineWidth', 2);
%plot(t_nmpc, x_solution_nmpc(3, :), 'LineWidth', 2);
plot(t_mpc_with_err, x_sol_mpc_with_error(3, :), 'LineWidth', 2);
xlabel('sec');
ylabel('rad/s');
title('Angualr Velocity on q1 plane');
legend('OCP with error', 'OCP Ideal Val', 'NMPC with error','Location','southeast');

subplot(2, 2, 4);
axis([0, 3, -3/2*pi, 3/2*pi]);
hold on;
grid on;
plot(t, x_sol_ocp(4, :), 'LineStyle', '--', 'LineWidth', 2);
plot(t, x_sol_ocp_actual(4, :), 'LineWidth', 2);
%plot(t_nmpc, x_solution_nmpc(4, :), 'LineWidth', 2);
plot(t_mpc_with_err, x_sol_mpc_with_error(4, :), 'LineWidth', 2);
xlabel('sec');
ylabel('rad/s');
title('Angualr Velocity on q2 plane');
legend('OCP with error', 'OCP Ideal Val', 'NMPC with error','Location','southeast');

%% plot x-y-plane

x_solved = const.l1 .* cos(x_sol_ocp(1, :)) + const.l2 .* cos(x_sol_ocp(1, :) + x_sol_ocp(2, :));
y_solved = const.l1 .* sin(x_sol_ocp(1, :)) + const.l2 .* sin(x_sol_ocp(1, :) + x_sol_ocp(2, :));

x_real = const.l1 .* cos(x_sol_ocp_actual(1, :)) + const.l2 .* cos(x_sol_ocp_actual(1, :) + x_sol_ocp_actual(2, :));
y_real = const.l1 .* sin(x_sol_ocp_actual(1, :)) + const.l2 .* sin(x_sol_ocp_actual(1, :) + x_sol_ocp_actual(2, :));


x_euler = const.l1 .* cos(x_sol_mpc_with_error(1, :)) + const.l2 .* cos(x_sol_mpc_with_error(1, :) + x_sol_mpc_with_error(2, :));
y_euler = const.l1 .* sin(x_sol_mpc_with_error(1, :)) + const.l2 .* sin(x_sol_mpc_with_error(1, :) + x_sol_mpc_with_error(2, :));

figure;
fig2 = gcf;
fig2.PaperUnits = 'inches';
fig2.PaperPosition = [0, 0, 8, 8];

subplot(2, 1, 1);
axis([0, 3, -1, 1]);
hold on;
grid on;
plot(t, x_solved, 'LineStyle', '--', 'LineWidth', 2);
plot(t, x_real, 'LineWidth', 2);
plot(t_mpc_with_err, x_euler, 'LineWidth', 2);
xlabel('sec');
ylabel('m/sec');
title('Motion on x-plane');
legend('OCP with incorrect params', 'OCP Ideal Val', 'MPC with incorrect params','Location','southeast');

subplot(2, 1, 2);
axis([0, 3, -0.5, 1]);
hold on;
grid on;
plot(t, y_solved, 'LineStyle', '--', 'LineWidth', 2);
plot(t, y_real, 'LineWidth', 2);
plot(t_mpc_with_err, y_euler, 'LineWidth', 2);
xlabel('sec');
ylabel('m/sec');
title('Motion on y-plane');
legend('OCP with incorrect params', 'OCP Ideal Val', 'MPC with incorrect params','Location','southeast');


%print('model_xy','-dpng','-r0');
%% plot inputs
figure;
fig3 = gcf;
fig3.PaperUnits='inches';
fig3.PaperPosition = [0 0 8 8];
subplot(2, 1, 1);

axis([0, 3, -1000, 1200]);
hold on;
grid on;
stairs(t(2:length(t)), u_sol_ocp(1, :), 'LineWidth', 2);
stairs(t_mpc_with_err(2:length(t_mpc_with_err)), u_sol_mpc_with_error(1, :), 'LineWidth', 2);
xlabel('sec');
ylabel('N.m');
title('Applied Torque by U1');
legend('OCP with incorrect params',  'MPC with incorrect params');

subplot(2, 1, 2);
axis([0, 3, -1000, 500]);
hold on;
grid on;
stairs(t(2:length(t)), u_sol_ocp(2, :), 'LineWidth', 2);
stairs(t_mpc_with_err(2:length(t_mpc_with_err)), u_sol_mpc_with_error(2, :), 'LineWidth', 2);
xlabel('sec');
ylabel('N.m');
title('Applied Torque by U2');
legend('OCP with incorrect params', 'MPC with incorrect params');

print('model_input','-dpng','-r0');




%% ========================================================================
%  subfunctions
%  =========================================================================


function [x_sol, u_sol, t] = ocp_approach(sample_points,typ)
import casadi.*;

opti = casadi.Opti();
const = constants();


%% define start/end time and sampling rate
t_f = 3;
t_0 = 0;
N = sample_points;
timeSteps = (t_f - t_0) / N;

%% define variables
x = opti.variable(4, N + 1);
u = opti.variable(2, N);

%% objective function
t = t_0;


x_0 = const.x_0;
x_end = const.x_end;


for i = 1:N
    % Solving intaial value problem step by step using linear multi step
    % method and appplying constarint that help to stick to the ODE 
    if strcmp(typ, 'explicitEuler')
        x_next = explicitEuler_incorrect(x(:, i), u(:, i), timeSteps,t(end));
    elseif strcmp(typ, 'explicitRungeKutta')
        x_next = explicitRungeKutta_incorrect(x(:, i), u(:, i),timeSteps);
    elseif strcmp(typ, 'midpointstep')
        x_next = midpointstep_incorrect( x(:, i), u(:, i), timeSteps);
     elseif strcmp(typ, 'heun')
         x_next = heun_incorrect(x(:, i), u(:, i), timeSteps);
    end


    opti.subject_to(x(:,i+1) == x_next);
    t = [t, t(end) + timeSteps];

end

opti.minimize(t_f);

%% constraints formulation

opti.subject_to(x(:, 1) == x_0);
opti.subject_to(x(:, end) == x_end);
opti.subject_to(const.omega_lb <= x(3, :) <= const.omega_ub);
opti.subject_to(const.omega_lb <= x(4, :) <= const.omega_ub);
opti.subject_to(const.u_lb / 1000 <= u(1, :) <=  const.u_ub / 1000);
opti.subject_to(const.u_lb / 1000 <= u(2, :) <= const.u_ub / 1000);



%% initial guess
opti.set_initial(x, repmat([0;0;0;0], [1, N + 1]));
opti.set_initial(u, repmat([0;0],[1, N]));

%% solve
opti.solver('ipopt'); % Ipopt, a library for large-scale nonlinear optimization.

sol = opti.solve();

% opti.debug.value(x_next)
% opti.debug.value(x,opti.initial())
% opti.debug.show_infeasibilities()

x_sol = full(sol.value(x));
u_sol = full(sol.value(u)); % .* [1000; 1000];
 

end

function [x_mpc, u_mpc, t_mpc] = mpc(model_err, disturbance_err, std_dev)
import casadi.*;
const = constants();

%% OCP-Like
opti = casadi.Opti();

t_horizon = 1;
t_0 = 0;
N = 50;
t_s = (t_horizon - t_0) / N;
t_opt = 3;

x = opti.variable(4, N + 1);
u = opti.variable(2, N);

% J = 0;
% Q = 1*eye(4);
% R = 1*eye(2);

x_0 = opti.parameter(4, 1);
x_end = const.x_end;

for i = 1:N
    if(model_err == 1)
        x_next = explicitEuler_incorrect_mpc( x(:, i), u(:, i), t_s * t_opt);
    else
        x_next = explicitEuler(  x(:, i), u(:, i), t_s * t_opt);
    end
    
    opti.subject_to(x(:, i + 1) == x_next);
    
    %J = J + u(:, i)' * R * u(:, i) + (x(:, i + 1) - x_end)' * Q * (x(:, i + 1) - x_end);
end

opti.subject_to(x(:, 1) == x_0);
opti.subject_to(x(:, end) == x_end);
opti.subject_to(const.omega_lb <= x(3, :) <= const.omega_ub);
opti.subject_to(const.omega_lb <= x(4, :) <= const.omega_ub);
opti.subject_to( const.u_lb / 1000 <= u(1, :) <=  const.u_ub / 1000);
opti.subject_to( const.u_lb / 1000 <= u(2, :) <=  const.u_ub / 1000);

opti.subject_to(u(:, end) == 0);


opti.minimize(t_horizon);
opti.solver('ipopt');

%% MPC
t_end = 3;
N_nmpc = t_end / t_s;

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
        opti.set_initial(x, repmat([0; 0; 0; 0], 1, N + 1));
        opti.set_initial(u, repmat([0; 0], 1, N));
    else
        opti.set_initial(x, x_guess);
        opti.set_initial(u, u_guess);
    end
    
    sol = opti.solve();
    
    x_guess = [full(sol.value(x(:, 2:end))), [0; 0; 0; 0]];
    u_guess = [full(sol.value(u(:, 2:end))), [0; 0]];
    
    
    u_ = full(sol.value(u(:, 1)));
    if(disturbance_err == 1)
        noise = normrnd(0, std_dev, 4, 1);
        x_real = heun(t_mpc(end), x_real, u_, t_s * t_opt);
        x_ = x_real + noise;

    else
        x_real = heun(t_mpc(end), x_real, u_, t_s * t_opt);
        x_ = x_real;
    end
    
    opti.set_value(x_0, x_);
    
    x_result = [x_result, x_real];
    u_result = [u_result, u_];
    t_mpc = [t_mpc, i * t_s];
end

x_mpc = x_result;
u_mpc = u_result;

end

function x_next = explicitEuler_incorrect_mpc( x, u, h  )
%Solving an ODR using one-Step Method
% Computation is performed without iteration
 
x_next = x + h .* stateSpace_ODE_incorrect(1, x, u ); % revise Buchter Diagram

end

function x_next = explicitRungeKutta_incorrect( x, u, h  )
% % Fixed step Runge-Kutta 4th order integrator
k1 = stateSpace_ODE_incorrect( 1, x, u);
k2 = stateSpace_ODE_incorrect( 1 + h / 2, x + k1 * h / 2, u );
k3 = stateSpace_ODE_incorrect( 1 + h / 2 , x + k2 * h / 2, u );
k4 = stateSpace_ODE_incorrect( 1 + h, x + k3 * h, u);
% % RK4 steps per interval
x_next = x + (k1 + 2 * k2 + 2 * k3 + k4) * h / 6;
% compare single shooting casadi example    
end

function y = midpointstep_incorrect(x, u, h)
    halfh = h/2;
    z = x + halfh * stateSpace_ODE_incorrect(1, x, u);
    y = x + h * stateSpace_ODE_incorrect(1, z , u  );
end

function xf = heun_incorrect(x, u, h )

     xt = x + h * stateSpace_ODE_incorrect(1, x,u);

     xf = x + h/2 * (stateSpace_ODE_incorrect(1, x,u) + stateSpace_ODE_incorrect(1 + h, xt,u));
end


function xdot = stateSpace_ODE_incorrect(time_instant, X_Arg, Steer_Arg )
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
    
xdot = [X_Arg(3); X_Arg(4); Bq ^ (-1) * (maginified_u - g - C * X_Arg(3:4))] ;
    
end


function xf = heun(t, x, u, h)

     xt = x + h * stateSpace_ODE(t, x,u);
     xf = x + h/2 * (stateSpace_ODE(t, x,u) + stateSpace_ODE(t+h , xt ,u)); % revise Buchter Diagram
     
end

% 
% function x_next = explicitRungeKutta( x, u, h )
% % % Fixed step Runge-Kutta 4th order integrator
% k1 = stateSpace_ODE( x, u, 1);
% k2 = stateSpace_ODE( x + k1 * h / 2, u ,  1  );
% k3 = stateSpace_ODE( x + k2 * h / 2, u , 1 );
% k4 = stateSpace_ODE( x + k3 * h, u , 1 );
% % % RK4 steps per interval
% x_next = x + (k1 + 2 * k2 + 2 * k3 + k4) * h / 6; % revise Buchter Diagram
% % compare single shooting casadi example    
% end