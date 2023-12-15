addpath(fullfile('..', 'src'));

close all
clear all
clc
%% 
Ts = 1/20;
rocket = Rocket(Ts);


%% Simulate the system:

% d1 = 0;
% d2 = 0;
% Pavg =90;
% Pdiff = 2;
% u = [d1, d2, Pavg, Pdiff]'; % (Assign appropriately)
% [b_F, b_M] = rocket.getForceAndMomentFromThrust(u);
% 
% w = zeros(3,1)';
% phi = zeros(3,1)';
% v = zeros(3,1)';
% p = zeros(3,1)';
% x = [w, phi, v, p]'; % (Assign appropriately)
% 
% 
% for k = 1:100
%     [x_dot, ~] = f(rocket, x, u);
% 
%     x = x + x_dot*Ts
% 
% end

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

%% Linearization around trim point
rocket = Rocket(Ts);
[xs, us] = rocket.trim() % Compute steady−state for which 0 = f(xs,us)
sys = rocket.linearize(xs, us) % Linearize the nonlinear model about trim point
% It can be seen that all the states at trim point are null. 
% All the inputs, apart from Pavg are null.
% This has a physical meaning, nbecause trim point is located at origin,
% zero velocities and accelerations. Pdiff is zero, meaning no "rollio".
% Only non zero value is Pavg, because it is needed to compensate for
% gravity.

%% Decomposition into subsystems
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us)

%% TODO 3.1 Design MPC controller X
H = 0.6; % Horizon length in seconds
%????????????????da giustificare, presa più piccola di settling time,
%perché altrimenti open loop e closed loop coincidono


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


% %% 
% x_x = [];
% u_u =[];        % input delta_2
% x_x(:,1) = [0 0 0 3]'; % (wy, beta, vx, x) Initial state
% X_opt =[];
% U_opt  = [];
% final_iteration = 10/Ts; %number of iteration to have 10s of simulations
% for k = 1:final_iteration-1
%     X_opt =[];
%     U_opt  = [];
%     [u, T_opt, X_opt, U_opt] = mpc_x.get_u(x_x(:,k));
% 
%     %x_x(:,k+1) = X_opt(:,1);
%     u_u(:,k) = U_opt(:,1);
%     x_x(:,k+1) = sys_x.A*x_x(:,k)+ sys_x.B*u_u(:,k);
% 
% end
% 
% u_u(:,end+1) = NaN;
% % Account for linearization point
% % Since all states and input at trim point are null, no shift is applied
% 
% ph = rocket.plotvis_sub(1:k+1, x_x, u_u, sys_x, xs, us); % Plot as usual

%% TODO 3.1 Design MPC controller Y
H = 0.6; % Horizon length in seconds
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
Tf = 30;
[T, Y_sub, U_sub] = rocket.simulate_f(sys_y, x_y, Tf, @mpc_y.get_u, 0);
ph = rocket.plotvis_sub(T, Y_sub, U_sub, sys_y, xs, us);

% %% 
% x_y = [];
% u_u =[];
% x_y(:,1) = [0 0 0 3]'; % (wx, alpha, vy, y) Initial state
% 
% for k = 1:100
%     [u, T_opt, Y_opt, U_opt] = mpc_y.get_u(x_y(:,k));
% 
%     x_y(:,k+1) = Y_opt(:,1);
%     u_u(:,k) = U_opt(:,k);
%     disp(k)
% end
% 
% u_u(:,end+1) = NaN;
% ph = rocket.plotvis_sub(1:k+1, x_y, u_u, sys_x, xs, us); % Plot as usual

%% TODO 3.1 Design MPC controller Z
H = 0.6; % Horizon length in seconds
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

% %% 
% x_z = [];
% u_u =[];
% x_z(:,1) = [0 3]'; % (w, phi, v, x) Initial state
% 
% for k = 1:100
%     [u, T_opt, Z_opt, U_opt] = mpc_z.get_u(x_z(:,k));
% 
%     x_z(:,k+1) = Z_opt(:,1);
%     u_u(:,k) = U_opt(:,k);
%     disp(k)
% end
% 
% u_u(:,end+1) = NaN;
% ph = rocket.plotvis_sub(1:k+1, x_z, u_u, sys_x, xs, us); % Plot as usual
%% TODO 3.1 Design MPC controller roll
H = 0.6; % Horizon length in seconds
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

% %% 
% x_roll = [];
% u_u =[];
% x_roll = [0 deg2rad(30)]'; % (wz, gamma) Initial state
% 
% for k = 1:100
%     [u, T_opt, Z_opt, U_opt] = mpc_roll.get_u(x_roll(:,k));
% 
%     x_roll(:,k+1) = Z_opt(:,1);
%     u_u(:,k) = U_opt(:,k);
%     disp(k)
% end
% 
% u_u(:,end+1) = NaN;
% ph = rocket.plotvis_sub(1:k+1, x_roll, u_u, sys_x, xs, us); % Plot as usual
% 