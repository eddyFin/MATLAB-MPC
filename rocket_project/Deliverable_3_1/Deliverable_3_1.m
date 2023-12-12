addpath(fullfile('..', 'src'));

close all
clear all
clc

%% TODO: This file should produce all the plots for the deliverable

%% Simulate the system:
%git test

Ts = 1/20;
rocket = Rocket(Ts);
d1 = 0;
d2 = 0;
Pavg = 90;
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


rocket = Rocket(Ts);
Tf = 20.0; % Simulation end time
x0 = [deg2rad([2 -2 0, -2 2 0]), 0 0 0, 0 0 0]'; % (w, phi, v, p) Initial state
u = [deg2rad([0 0]), 100, 0 ]'; % (d1 d2 Pavg Pdiff) Constant input
[T, X, U] = rocket.simulate(x0, Tf, u); % Simulate unknown, nonlinear model
rocket.anim_rate = 1.0; % Visualize at 1.0x real−time
rocket.vis(T, X, U);

%% 
rocket = Rocket(Ts);
[xs, us] = rocket.trim() % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us) % Linearize the nonlinear model about trim point
