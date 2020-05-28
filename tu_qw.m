function [x, P] = tu_qw(x, P, omega, T, Rw)
% EKF time update step
if any(isnan(omega))
    x = x;
    P = P;
else
    
% Gyroscope measurement noise covariance (estimated from stationary data)
sigma_v = 0.0014;
Rv = diag([sigma_v^2 sigma_v^2 sigma_v^2]);

% Process matrices
F = T/2*Somega(omega) + eye(4);
F_tilde  = F;
G = T/2*Sq(x);
G_tilde = G;

% Prediction step
xp = F*x;
Pp = F_tilde*P*F_tilde' + G_tilde*Rw*G_tilde';



    % Update step
    % Equation: yk = H*xk + B*xkmin1_kmin1 + vk
    H = 2/T*pinv(Sq(x));
    B = -H;
    Bx = B*x;
    yhat = H*xp + Bx;

    S = H*Pp*H' + B*P*B' + Rv;
    K = Pp*H'*(S^-1);

    x = xp + K*(omega - yhat);
    P = Pp - K*S*K';
end

end