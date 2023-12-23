addpath(fullfile('..', 'src'));

close all
clear all
clc

%% TODO: This file should produce all the plots for the deliverable
Ts = 1/20;
rocket = Rocket(Ts);


%% Linearization around trim point
rocket = Rocket(Ts);
[xs, us] = rocket.trim(); % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us); % Linearize the nonlinear model about trim point
% It can be seen that all the states at trim point are null. 
% All the inputs, apart from Pavg are null.
% This has a physical meaning, nbecause trim point is located at origin,
% zero velocities and accelerations. Pdiff is zero, meaning no "rollio".
% Only non zero value is Pavg, because it is needed to compensate for
% gravity.

%% Decomposition into subsystems
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

%% TODO 3.1 Design MPC controller X
H = 1; % Horizon length in seconds

mpc_x = MpcControl_x(sys_x, Ts, H);
% Get control input ( x is the index of the subsystem here)
x_x = [0 0 0 3]'; % (wy, beta, vx, x) Initial state
    
% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x_x);
U_opt(:,end+1) = NaN;
% Account for linearization point
% Since all states and input at trim point are null, no shift is applied

% Plot the open loop trajectory
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us); 
%% Receiding horizon approach for computing the closed loop trajectory:
Tf = 10;
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x_x, Tf, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);

%% TODO 3.1 Design MPC controller Y
H = 1; % Horizon length in seconds
mpc_y = MpcControl_y(sys_y, Ts, H);
% Get control input ( x is the index of the subsystem here)
x_y = [0 0 0 3]'; % (wx, alpha, vy, y) Initial state


% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u, T_opt, Y_opt, U_opt] = mpc_y.get_u(x_y);
U_opt(:,end+1) = NaN;
% Account for linearization point
% Since all states and input at trim point are null, no shift is applied

% Plot the open loop trajectory
ph = rocket.plotvis_sub(T_opt, Y_opt, U_opt, sys_y, xs, us); 

%% Receiding horizon approach for computing the closed loop trajectory:
Tf = 10;
[T, Y_sub, U_sub] = rocket.simulate_f(sys_y, x_y, Tf, @mpc_y.get_u, 0);
ph = rocket.plotvis_sub(T, Y_sub, U_sub, sys_y, xs, us);

%% TODO 3.1 Design MPC controller Z
H = 1; % Horizon length in seconds
mpc_z = MpcControl_z(sys_z, Ts, H);
% Get control input ( x is the index of the subsystem here)
x_z = [0 3]'; % (vz, z) Initial state


% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u, T_opt, Z_opt, U_opt] = mpc_z.get_u(x_z);
U_opt(:,end+1) = NaN;
% Account for linearization point
% no shift to be applied to Z_opt
U_opt = U_opt+us(3)*ones(1,size(U_opt,2));

% Plot the open loop trajectory
ph = rocket.plotvis_sub(T_opt, Z_opt, U_opt, sys_z, xs, us); 

%% Receiding horizon approach for computing the closed loop trajectory:
Tf = 10;
[T, Z_sub, U_sub] = rocket.simulate_f(sys_z, x_z, Tf, @mpc_z.get_u, 0);
ph = rocket.plotvis_sub(T, Z_sub, U_sub, sys_z, xs, us);


%% TODO 3.1 Design MPC controller roll
H = 1; % Horizon length in seconds
mpc_roll = MpcControl_roll(sys_roll, Ts, H);
% Get control input ( x is the index of the subsystem here)
x_roll = [0 deg2rad(30)]'; % (wz, gamma) Initial state


% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u, T_opt, Gamma_opt, U_opt] = mpc_roll.get_u(x_roll);
U_opt(:,end+1) = NaN;
% Account for linearization point
% Since all states and input at trim point are null, no shift is applied

% Plot the open loop trajectory
ph = rocket.plotvis_sub(T_opt, Gamma_opt, U_opt, sys_roll, xs, us); 
%% Receiding horizon approach for computing the closed loop trajectory:
Tf = 10;
[T, Gamma_sub, U_sub] = rocket.simulate_f(sys_roll, x_roll, Tf, @mpc_roll.get_u, 0);
ph = rocket.plotvis_sub(T, Gamma_sub, U_sub, sys_roll, xs, us);