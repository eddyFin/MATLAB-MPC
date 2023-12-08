function [F,L] = pendulum_discrete(time_step)
%% Compute discrete-time dynamics
import casadi.*

% Define variables
th = SX.sym('th');
w  = SX.sym('w');
x = [th;w];
u  = SX.sym('u');

% Define system dynamics
xdot = [w;-10*sin(th) + u];

% Objective term: to be minimized
L = 100*(th - pi)^2 + w^2 + u^2;

% Continuous time dynamics
f = Function('f', {x, u}, {xdot, L});

% Formulate discrete time dynamics with Runge-Kutta 4 integrator
DT = time_step;
X0 = MX.sym('X0', 2);
U = MX.sym('U');
X = X0;
Q = 0;
[k1, k1_q] = f(X, U);
[k2, k2_q] = f(X + DT/2 * k1, U);
[k3, k3_q] = f(X + DT/2 * k2, U);
[k4, k4_q] = f(X + DT * k3, U);
X = X + DT/6*(k1   + 2*k2   + 2*k3   + k4);
Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
% F = Function('F', {X0, U}, {X, Q}, {'x0','u'}, {'xp', 'obj'});
F = Function('F', {X0, U}, {X});
L = Function('L', {X0, U}, {Q});
end

