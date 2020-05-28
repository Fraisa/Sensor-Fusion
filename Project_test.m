% SSY345 Project


% Task 1:


% Task 2: Collect meas. data and compute mean and cov. of sensors data.

figure(1)
plot(t, y_gyr(1,:),'b')
hold on
grid on
plot(t, y_gyr(2,:),'r')
plot(t, y_gyr(3,:),'g')
hold off
title('Gyroscope')

figure(2)
plot(t, y_acc(1,:),'b')
hold on
grid on
plot(t, y_acc(2,:),'r')
plot(t, y_acc(3,:),'g')
hold off
xlabel('Time [s]'), ylabel('Magnitude [m/s^2]')
title('Accelerometer')

figure(3)
plot(t, y_mag(1,:),'b')
hold on
grid on
plot(t, y_mag(2,:),'r')
plot(t, y_mag(3,:),'g')
hold off
xlabel('Time [s]'), ylabel('Magnitude [{\mu}T]')
title('Magnetometer')


mean_gyr = mean(y_gyr(:,~isnan(y_gyr(1,:)))')'
var_gyr = var(y_gyr(:,~isnan(y_gyr(1,:)))')';
var_gyr_tot = sum(var_gyr)

cov_gyr(:,:) = zeros(3,3);
for k = 1:length(var_gyr(1,:))
    cov_gyr = cov_gyr + (y_gyr(:,k) - mean_gyr)*(y_gyr(:,k) - mean_gyr)';
end
cov_gyr

mean_acc = mean(y_acc(:,~isnan(y_acc(1,:)))')'
var_acc = var(y_acc(:,~isnan(y_acc(1,:)))')';
var_acc_tot = sum(var_acc)

mean_mag = mean(y_mag(:,~isnan(y_mag(1,:)))')'
var_mag = var(y_mag(:,~isnan(y_mag(1,:)))')';
var_mag_tot = sum(var_mag)

%% Plot histograms
figure(4)
subplot(3,1,1)
histogram(y_gyr(1,:),'Normalization','pdf')
grid on
title('Gyroscope histogram')
xlabel('x magnitude [rad/s]')
subplot(3,1,2)
histogram(y_gyr(2,:),'Normalization','pdf')
grid on
xlabel('y magnitude [rad/s]')
subplot(3,1,3)
histogram(y_gyr(3,:),'Normalization','pdf')
grid on
xlabel('z magnitude [rad/s]')

figure(5)
subplot(3,1,1)
histogram(y_acc(1,:),'Normalization','pdf')
grid on
title('Accelerometer histogram')
xlabel('x magnitude [m/s^2]')
subplot(3,1,2)
histogram(y_acc(2,:),'Normalization','pdf')
grid on
xlabel('y magnitude [m/s^2]')
subplot(3,1,3)
histogram(y_acc(3,:),'Normalization','pdf')
grid on
xlabel('z magnitude [m/s^2]')

figure(6)
subplot(3,1,1)
histogram(y_mag(1,:),'Normalization','pdf')
grid on
title('Magnetometer histogram')
xlabel('x magnitude [{\mu}T]')
subplot(3,1,2)
histogram(y_mag(2,:),'Normalization','pdf')
grid on
xlabel('y magnitude [{\mu}T]')
subplot(3,1,3)
histogram(y_mag(3,:),'Normalization','pdf')
grid on
xlabel('z magnitude [{\mu}T]')
%% Convert data to fit format stuff

T = timetable2table(y_mag);
Mag = table2array(T(:,2:end))
y_mag = Mag'
%%
fs = 100
T = (0:length(y_acc(1,:))-1)*1/fs
%%
T = timetable2table(AngularVelocity);
Gyr = table2array(T(:,2:end));
y_gyr = Gyr';

T = timetable2table(Acceleration);
Acc = table2array(T(:,2:end));
y_acc = Acc';

T = timetable2table(MagneticField);
Mag = table2array(T(:,2:end));
y_mag = Mag';

y_gyr = y_gyr(:,1:1200);
y_acc = y_acc(:,1:1200);
y_mag = y_mag(:,1:1200);
t = (1:1200)/fs;















