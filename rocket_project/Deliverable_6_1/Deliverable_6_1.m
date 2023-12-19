addpath(fullfile('..', 'src'));

close all
clear all
clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20;
rocket = Rocket(Ts);

rocket.delay = 0;

H = 6; % Horizon length in seconds
nmpc = NmpcControl(rocket, H);
% MPC reference with default maximum roll = 15 deg
%ref = @(t_, x_) ref_TVC(t_);
ref = [0.5, 0, 1, deg2rad(5)]';
%ref = [0, 0, 0, 0]';

% % MPC reference with specified maximum roll = 50 deg
% roll_max = deg2rad(50);
% ref = @(t_, x_) ref_TVC(t_, roll_max);

% Evaluate once and plot optimal open−loop trajectory,
% pad last input to get consistent size with time and state
x = zeros(12,1);
x(10:12) = 3;

[u, T_opt, X_opt, U_opt] = nmpc.get_u(x, ref);
U_opt(:,end+1) = nan;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref);
%%

Tf =20;
[T, X, U, Ref] = rocket.simulate(x, Tf, @nmpc.get_u, ref);

% Visualize
rocket.anim_rate = 20; % Increase this to make the animation faster
ph = rocket.plotvis(T, X, U, Ref);
