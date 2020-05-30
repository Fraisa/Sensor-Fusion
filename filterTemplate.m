function [xhat, meas] = filterTemplate(calAcc, calGyr, calMag)
% FILTERTEMPLATE  Filter template
%
% This is a template function for how to collect and filter data
% sent from a smartphone live.  Calibration data for the
% accelerometer, gyroscope and magnetometer assumed available as
% structs with fields m (mean) and R (variance).
%
% The function returns xhat as an array of structs comprising t
% (timestamp), x (state), and P (state covariance) for each
% timestamp, and meas an array of structs comprising t (timestamp),
% acc (accelerometer measurements), gyr (gyroscope measurements),
% mag (magnetometer measurements), and orint (orientation quaternions
% from the phone).  Measurements not availabe are marked with NaNs.
%
% As you implement your own orientation estimate, it will be
% visualized in a simple illustration.  If the orientation estimate
% is checked in the Sensor Fusion app, it will be displayed in a
% separate view.
%
% Note that it is not necessary to provide inputs (calAcc, calGyr, calMag).

%% Setup necessary infrastructure
  import('com.liu.sensordata.*');  % Used to receive data.

  % Filters used
  magFilt = true;
  accFilt = true;
  
  
  %% Filter settings
  t0 = [];  % Initial time (initialize on first data received)
  nx = 4;   % Assuming that you use q as state variable.
  % Add your filter settings here.
  T = 1/25;
  % Gyroscope covariances
  sigma_w = 1/10; % was 0.01 
  Rw = diag([sigma_w^2 sigma_w^2 sigma_w^2]);
  sigma_v = 0.0014;
  Rv = diag([sigma_v^2 sigma_v^2 sigma_v^2]);
  gyr_old = [0; 0; 0];

  % Accelerometer parameters and covariances
  g_abs = 9.82;   % 9.8908 from estimate using stationary data.
  g_lim = [0.8 1.2]*g_abs;    % +- 20% marginal
  sigma_a = 0.3; % From variance estimate using stationary data.
  Ra = diag([sigma_a^2 sigma_a^2 sigma_a^2]);

  % Magnetometer parameters and covariances
  m_abs = 39.1103;   % 39.1103 from estimate using stationary data.
  m_lim = [0.6 1.4]*m_abs;    % +- 20% marginal
  sigma_m = 0.9; % From variance estimate using stationary data.s
  Rm = diag([sigma_m^2 sigma_m^2 sigma_m^2]);
  
  
  % Current filter state.
  x = [1; 0; 0; 0];
  P = eye(nx, nx);

  % Saved filter states.
  xhat = struct('t', zeros(1, 0),...
                'x', zeros(nx, 0),...
                'P', zeros(nx, nx, 0));

  meas = struct('t', zeros(1, 0),...
                'acc', zeros(3, 0),...
                'gyr', zeros(3, 0),...
                'mag', zeros(3, 0),...
                'orient', zeros(4, 0));
  try
    %% Create data link
    server = StreamSensorDataReader(3400);
    % Makes sure to resources are returned.
    sentinel = onCleanup(@() server.stop());

    server.start();  % Start data reception.

    % Used for visualization.
    figure(1);
    subplot(1, 2, 1);
    ownView = OrientationView('Own filter', gca);  % Used for visualization.
    googleView = [];
    counter = 0;  % Used to throttle the displayed frame rate.

    %% Filter loop
    while server.status()  % Repeat while data is available
      % Get the next measurement set, assume all measurements
      % within the next 5 ms are concurrent (suitable for sampling
      % in 100Hz).
      data = server.getNext(5);

      if isnan(data(1))  % No new data received
        continue;        % Skips the rest of the look
      end
      t = data(1)/1000;  % Extract current time

      if isempty(t0)  % Initialize t0
        t0 = t;
      end
      
      gyr = data(1, 5:7)';
      % Gyroscope 
      if ~any(isnan(gyr))
          w = gyr;
          x_tmp = x;
          P_tmp = P;
          [x, P] = tu_qw(x, P, gyr, T, Rw);
          [x, P] = mu_normalizeQ(x, P);
          x_kmin1 = x_tmp;
          P_kmin1 = P_tmp;
      else
%           x_tmp = x;
%           P_tmp = P;
%           [w, x, P] = tu_qw_pred(x, P, x_kmin1, P_kmin1, w, T, Rw);
%           [x, P] = mu_normalizeQ(x, P);
%           x_kmin1 = x_tmp;
%           P_kmin1 = P_tmp;
      end
      

      
      acc = data(1, 2:4)';
      if accFilt
          if ~any(isnan(acc))  % Acc measurements are available.
          % Approximate g, skip update if "outlier"
          a_abs = sqrt(sum(acc.^2));
              if (a_abs > g_lim(1)) && (a_abs < g_lim(2))
                  g0 = [0; 0; a_abs];
                  [x, P] = mu_g(x, P, acc, Ra, g0);
                  [x, P] = mu_normalizeQ(x, P);
              end
          end
      end
      
      

      mag = data(1, 8:10)';
      if magFilt
          if ~any(isnan(mag))  % Mag measurements are available.
              % Approximate m, skip update if "outlier"
              m_abs = sqrt(sum(mag.^2));
              if (m_abs > m_lim(1)) && (m_abs < m_lim(2))
                  m0 = [0; 12; -36.09]; m0 = m0/norm(m0);
                  [x, P] = mu_m(x, P, mag, Rm, m0);
                  [x, P] = mu_normalizeQ(x, P);
              end
          end
      end

      orientation = data(1, 18:21)';  % Google's orientation estimate.
      

      % Visualize result
      if rem(counter, 10) == 0
        setOrientation(ownView, x(1:4));
        title(ownView, 'OWN', 'FontSize', 16);
        if ~any(isnan(orientation))
          if isempty(googleView)
            subplot(1, 2, 2);
            % Used for visualization.
            googleView = OrientationView('Google filter', gca);
          end
          setOrientation(googleView, orientation);
          title(googleView, 'GOOGLE', 'FontSize', 16);
        end
      end
      counter = counter + 1;

      % Save estimates
      xhat.x(:, end+1) = x;
      xhat.P(:, :, end+1) = P;
      xhat.t(end+1) = t - t0;

      meas.t(end+1) = t - t0;
      meas.acc(:, end+1) = acc;
      meas.gyr(:, end+1) = gyr;
      meas.mag(:, end+1) = mag;
      meas.orient(:, end+1) = orientation;
      
    end
  catch e
    fprintf(['Unsuccessful connecting to client!\n' ...
      'Make sure to start streaming from the phone *after*'...
             'running this function!']);
  end
end


%% "tu_qw"
% <include>tu_qw.m</include>
%% "tu_qw_pred"
% <include>tu_qw_pred.m</include>
%% "mu_g"
% <include>mu_g.m</include>
%% "mu_m"
% <include>mu_m.m</include>
%% "Project_Implementation"
% <include>Project_Implementation.m</include>

