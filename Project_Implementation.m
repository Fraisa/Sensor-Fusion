% SSY345 Project Implementation

% SECTION 4.1
% Task 2) See file Project_test.m. Measurement data loaded below.

load('data_FrassePhone_stationary.mat');

%% SECTION 4.2
% Task 3) Design the EKF time update step
% Implemented in file "tu_qw.m".

% Time between samples
T = 1/fs;

% Sizes
K = length(t);
n = 4;

% Gyroscope covariances
sigma_w = 0.01;
Rw = diag([sigma_w^2 sigma_w^2 sigma_w^2]);
sigma_v = 0.0014;
Rv = diag([sigma_v^2 sigma_v^2 sigma_v^2]);

% Accelerometer parameters and covariances
g_abs = 9.82;   % 9.8908 from estimate using stationary data.
g_lim = [0.9 1.1]*g_abs;    % +- 10% marginal
sigma_a = 0.03; % From variance estimate using stationary data.
Ra = diag([sigma_a^2 sigma_a^2 sigma_a^2]);

% Magnetometer parameters and covariances
m_abs = 39.1103;   % 39.1103 from estimate using stationary data.
m_lim = [0.9 1.1]*m_abs;    % +- 10% marginal
sigma_m = 0.5; % From variance estimate using stationary data.s
Rm = diag([sigma_m^2 sigma_m^2 sigma_m^2]);

% Initial estimates and covariances
x_0 = [0; 0; 1; 0];
P_0 = diag([1 1 1 1]);

% Visualisation stuff
figure(1)
ownView = OrientationView('Own filter', gca);

xw = zeros(4,K);
Pw = zeros(4,4,K);
xw(:,1) = x_0; 
Pw(:,:,1) = P_0;

xa = zeros(4,K);
Pa = zeros(4,4,K);
xm = zeros(4,K);
Pm = zeros(4,4,K);

x = zeros(4,K);
P = zeros(4,4,K);
NaN_gyr = 1;
NaN_acc = 1;
for k = 2:K
    
    % Gyroscope measurement computations
    % Prediction:
    [xw(:,k), Pw(:,:,k)] = tu_qw(xw(:,k-1), Pw(:,:,k-1), y_gyr(:,k), T, Rw);

    % Include normalization of quaternion before using... where?
    [xw(:,k), Pw(:,:,k)] = mu_normalizeQ(xw(:,k), Pw(:,:,k));
    
    % Accelerometer computations
    % Measurement update:
    if ~isnan(y_acc(:,k))
        % Approximate g, skip update if "outlier"
        a_abs = sqrt(sum(y_acc(:,k).^2));
        if (a_abs > g_lim(1)) && (a_abs < g_lim(2))
            g0 = [0; 0; a_abs];
            [xa(:,k), Pa(:,:,k)] = mu_g(xw(:,k), Pw(:,:,k), y_acc(:,k), Ra, g0);
            [xa(:,k), Pa(:,:,k)] = mu_normalizeQ(xa(:,k), Pa(:,:,k));
        else
            xa(:,k) = xw(:,k);
            Pa(:,:,k) = Pw(:,:,k);
        end
    else
        xa(:,k) = xw(:,k);
        Pa(:,:,k) = Pw(:,:,k);
    end
    
    % Magnetometer computations
    % Measurement update:
    if ~isnan(y_mag(:,k))
        % Approximate m, skip update if "outlier"
        m_abs = sqrt(sum(y_mag(:,k+1).^2));
        if (m_abs > m_lim(1)) && (m_abs < m_lim(2))
            m0 = [0; 19.18; -34.09];
            [xm(:,k), Pm(:,:,k)] = mu_m(xa(:,k), Pa(:,:,k), y_mag(:,k), Rm, m0);
            [xm(:,k), Pm(:,:,k)] = mu_normalizeQ(xm(:,k), Pm(:,:,k));
        elseif ~isnan(y_acc(:,k))
            xm(:,k) = xa(:,k);
            Pm(:,:,k) = Pa(:,:,k);
        else
            xm(:,k) = xw(:,k);
            Pm(:,:,k) = Pw(:,:,k);
        end
    elseif ~isnan(y_acc(:,k))
        xm(:,k) = xa(:,k);
        Pm(:,:,k) = Pa(:,:,k);
    else
        xm(:,k) = xw(:,k);
        Pm(:,:,k) = Pw(:,:,k);
    end
    
    % Combine accelerometer and magnetometer information
    
    
    
    
    
    % Update:
    % Temporary for only gyroscope (Task 5)
%     x(:,k+1) = xwp(:,k+1);
%     P(:,:,k+1) = Pwp(:,:,k+1);
    
%     x(2:4,k+1) = xa(:,k+1);
%     P(2:4,2:4,k+1) = Pa(:,:,k+1);
%     [x(:,k+1),P(:,:,k+1)] = mu_normalizeQ(xwp(:,k+1), Pwp(:,:,k+1));
    
    setOrientation(ownView, xm(:,k));
    title(ownView, 'OWN', 'FontSize', 16);
    pause(0.01)
end









