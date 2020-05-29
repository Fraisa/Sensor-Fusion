function [x, P] = tu_qw_pred(x, P, omega, T, Rw)
% EKF time update step
    
% Gyroscope measurement noise covariance (estimated from stationary data)
sigma_v = 0.0014;
Rv = diag([sigma_v^2 sigma_v^2 sigma_v^2]);

% Process matrices
F = T/2*Somega(omega) + eye(4);
F_tilde  = F;
G = T/2*Sq(x);
G_tilde = G;

% Prediction step
x = F*x;
P = F_tilde*P*F_tilde' + G_tilde*Rw*G_tilde';


end