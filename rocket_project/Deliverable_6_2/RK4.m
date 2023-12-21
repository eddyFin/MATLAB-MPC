function [x_next] = RK4(X,U,h,rocket)
%
% Inputs : 
%    X, U current state and input
%    h    sample period
%    f    continuous time dynamics f(x,u)
% Returns
%    State h seconds in the future
%

% Runge-Kutta 4 integration
% write your function here


[k1] = rocket.f(X, U);
[k2] = rocket.f(X + h/2 * k1, U);
[k3] = rocket.f(X + h/2 * k2, U);
[k4] = rocket.f(X + h * k3, U);
x_next = X +h*(k1/6 + k2/3 + k3/3 + k4/6);

end