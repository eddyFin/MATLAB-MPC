addpath(fullfile('..', 'src'));

close all
clear all
clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20;
rocket = Rocket(Ts);

rocket.delay = 0;

H = 1; % Horizon length in seconds

% Constant reference
nmpc = NmpcControl(rocket, H);
ref = [0.5, 0, 1, deg2rad(5)]';
%ref = [0, 0, 0, 0]';

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
x = zeros(12,1);
%x(10:12) = 3;
%x = [ 3 3 3 0]';

% Open loop trajectory
[u, T_opt, X_opt, U_opt] = nmpc.get_u(x, ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref);

% Closed loop trajectory
Tf = 10;
[T, X, U, Ref] = rocket.simulate(x, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 20; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);


%%

Tf =30;
% MPC reference with default maximum roll = 15 deg
ref = @(t_, x_) ref_TVC(t_);
[T, X, U, Ref] = rocket.simulate(x, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 20; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
%%
% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref = @(t_, x_) ref_TVC(t_, roll_max);

[T, X, U, Ref] = rocket.simulate(x, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 20; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
%% Comparison with linear MPC
[xs, us] = rocket.trim(); % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us); % Linearize the nonlinear model about trim point


%Decomposition into subsystems
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

H =2; % Horizon length in seconds


mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% Merge four sub−system controllers into one full−system controller
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);


% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref = @(t_, x_) ref_TVC(t_, roll_max);

% Simulate
[T, X, U, Ref] = rocket.simulate(x, Tf, @mpc.get_u, ref);
% Visualize
rocket.anim_rate = 20; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
