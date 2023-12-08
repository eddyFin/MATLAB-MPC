% function pendulum(UMAX)
% if nargin < 1, UMAX = 5; end
clear
import casadi.*

UMAX = 5;


%% Formulate integrator (discrete-time model)

T = 5; % Time horizon
N = 100; % number of control intervals

[F,L] = pendulum_discrete(T/N);

%% Define the optimization problem

opti = casadi.Opti(); % Optimization problem

% Variables
u  = opti.variable(1, N);
x  = opti.variable(2, N+1);

x0 = opti.parameter(2, 1);

% Dynamic constraints
obj = 0;
for i = 1:N
  opti.subject_to(x(:, i+1) == F(x(:,i), u(:,i)));
  obj = obj + L(x(:,i), u(:,i));
end

% Input constraints
opti.subject_to(-UMAX <= u <= UMAX);

% Initial conditions
opti.subject_to(x(:,1) == x0);

% Objective
opti.minimize(obj);


%% Solve
opti.set_value(x0, [0;0]);
opti.solver('ipopt');
sol = opti.solve();

%% Plot
figure(1); clf
plot(sol.value(x)','o-')
grid on
legend('Angle', 'Velocity')

figure(2); clf
plot(sol.value(u),'o-')

