function [x, P] = mu_m(x, P, y, Rm, m0)
% EKF acclerometer measurement update

% Measurement matrices
hx = Qq(x)'*m0;
[dQ0, dQ1, dQ2, dQ3] = dQqdq(x);
Jhx = [dQ0'*m0 dQ1'*m0 dQ2'*m0 dQ3'*m0];

% Measurement update
S = Jhx*P*Jhx' + Rm;
K = P*Jhx'*(S^-1);

x = x + K*(y - hx);
P = P - K*S*K';

end