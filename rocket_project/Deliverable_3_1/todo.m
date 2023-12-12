addpath(fullfile('..', 'src'));

close all
clear all
clc
%% Simulate the system:
%git test

Ts = 1/20;
rocket = Rocket(Ts);

d1 = 0;
d2 = 0;
Pavg =90;
Pdiff = 2;
u = [d1, d2, Pavg, Pdiff]'; % (Assign appropriately)
[b_F, b_M] = rocket.getForceAndMomentFromThrust(u);

w = zeros(3,1)';
phi = zeros(3,1)';
v = zeros(3,1)';
p = zeros(3,1)';
x = [w, phi, v, p]'; % (Assign appropriately)


for k = 1:100
    [x_dot, ~] = f(rocket, x, u);

    x = x + x_dot*Ts

end

%% Simulate the system with functions provided:

% 
% rocket = Rocket(Ts);
% Tf = 5.0; % Simulation end time
% x0 = [deg2rad([0 -0 0, -0 0 0]), 0 0 0, 0 0 0]'; % (w, phi, v, p) Initial state
% u = [deg2rad([0 0]), 67, 0 ]'; % (d1 d2 Pavg Pdiff) Constant input
% [T, X, U] = rocket.simulate(x0, Tf, u); % Simulate unknown, nonlinear model
% rocket.anim_rate = 1.0; % Visualize at 1.0x real−time
% rocket.vis(T, X, U);
% 
% %Per salire verso l'alto: Pmean >~67

%% 
rocket = Rocket(Ts);
[xs, us] = rocket.trim() % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us) % Linearize the nonlinear model about trim point

[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us)

%% TODO 3.1
% Design MPC controller
H = 8; % Horizon length in seconds
mpc_x = MpcControl_x(sys_x, Ts, H);
% Get control input ( x is the index of the subsystem here)
x_x = [0 0 0 3]'; % (w, phi, v, x) Initial state
u_x = mpc_x.get_u(x_x);


% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x_x);
U_opt(:,end+1) = NaN;
% Account for linearization point

ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us); % Plot as usual
