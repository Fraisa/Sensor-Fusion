function [w, x, P] = tu_qw_pred(x, P, x_kmin1, P_kmin1, omega, T, Rw)
% EKF time update step

% Estimate omega
H = 2/T*pinv(Sq(x_kmin1));
B = -H;
Bx = B*x_kmin1;
w = H*x + Bx;

% recalculate Process matrices
F = T/2*Somega(w) + eye(4);
F_tilde  = F;
G = T/2*Sq(x);
G_tilde = G;


% Prediction step
x = F*x;
P = F_tilde*P*F_tilde' + G_tilde*Rw*G_tilde';


end