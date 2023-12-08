function [A] = jac_x(X0,U0,f)
% Write your jacobian function here using finite differences
delta = 0.1;
delta1 = [1;0]*delta;
delta2 = [0;1]*delta;
A1 = (f(X0+delta1,U0)-f(X0-delta1,U0))./(2*delta);
A2 = (f(X0+delta2,U0)-f(X0-delta2,U0))./(2*delta);
A = [A1,A2];

end