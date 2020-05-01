% This script focuses on simulating PID/MPC controllers for an Inverted Pendulum example.
% Chirath Hettiarachchi 

clear all;
close all;

% The target plants state space representation.(in Continous Time)
% dx_dt = Ax(t) + Bu(t)
% y(t) = Cx(t)
g = -10;
L = 2;
A_c = [0 1; g/L 0];
B_c = [0 ; 1];
C_c = [1 0];
D_c = zeros(1,1); %Assime U cant affect output y at same time.

% First we need to convert the continous to obtain discrete dynamics.
% x(k+1) = Ax(k) + Bu(k) ; 
% y(k) = Cx(k)
Ts   = 0.15; %sampling time.
sysc = ss(A_c,B_c,C_c,D_c);
sysd = c2d(sysc,Ts);
A_d = sysd.A;
B_d = sysd.B;
C_d = sysd.C;
D_d = sysd.D;

% Next we obtain augmented state space matrices (for the MPC controller).
[A,B,C] = get_AugmentedMatrices(A_d,B_d,C_d);
%%% Define MPC parameters. 
PredictionHorizon = 8;
ControlHorizon    = 4;

%%% Target for the controller. 
r = 180; %target value
Reference = ones(PredictionHorizon,1) * r;

%%% Initial conditions to the model.
x_state = [160; 0]; %initial conditions angular position = 160, ang vel = 0
y0  = C_d * x_state;
x_a = [x_state; y0];
u = 0;

%%% Simulate. 
tspan = 50;
plot_y = [];
plot_u = [];
plot_x = [];
err_hist = [0];
for time = 1:tspan
    
    plot_y = [plot_y y0];
    plot_u = [plot_u u];
    plot_x = [plot_x x_state];
    
    %%% MPC control inputs %%%
    delta_U = get_MPCinput(A,B,C,PredictionHorizon, ControlHorizon, Reference, x_a);
    u = u + delta_U(1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%% PID Controller. %%%%%%
    %err_hist = [err_hist (y0 - r)];
    %u = pid_controller(err_hist);
    %u = -1 * u;
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Updating the model with the the control action.
    x_prev  = x_state;
    x_state = A_d*x_state + B_d*u;
    y0      = C_d*x_state;
    x_a     = [x_state-x_prev;y0]; 
    
end

% plot the results
figure
subplot(3,1,1);
plot(plot_x(1,:)); hold on;
plot(plot_x(2,:))
hl =legend('${\theta}$ Angular Position','$\dot{\theta}$ Angular Velocity');
set(hl, 'Interpreter', 'latex');
xlabel('Time') 
title('System States X')
subplot(3,1,2);
plot(plot_y);
xlabel('Time') 
title('Measured output Y')
subplot(3,1,3);
stairs(plot_u)
xlabel('Time') 
title('Control Action')
suptitle('Inverted Pendulum')




