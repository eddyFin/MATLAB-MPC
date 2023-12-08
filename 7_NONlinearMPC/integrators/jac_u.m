function [B] = jac_u(X0,U0,f)
% Write your jacobian function here using finite differences
delta = 0.1;
delta1 = [delta;0];
delta2 = [0;delta];
B1 = (f(X0,U0+delta1)-f(X0,U0-delta1))./(2*delta);
B2 = (f(X0,U0+delta2)-f(X0,U0-delta2))./(2*delta);
B = [B1,B2];

end