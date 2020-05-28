function [x, P] = mu_g(x, P, y, Ra, g0)
% EKF accelerometer measurement update

% Measurement matrices
hx = Qq(x)'*g0;
[dQ0, dQ1, dQ2, dQ3] = dQqdq(x);
Jhx = [dQ0'*g0 dQ1'*g0 dQ2'*g0 dQ3'*g0];

% Measurement update
S = Jhx*P*Jhx' + Ra;
K = P*Jhx'*(S^-1);

x = x + K*(y - hx);
P = P - K*S*K';

end