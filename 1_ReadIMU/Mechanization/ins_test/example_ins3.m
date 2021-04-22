clear;
clc;
close all;
addpath('..\lib');
%% Read IMU
%load hi226_static_30s.mat
gyroReading_ = csvread('estelle_0412.csv');
gyroReading_(:,7) = gyroReading_(:,7) -  mean(gyroReading_(1500:4000,7)) - 9.7978;% g bias
gyroReading_(:,2:6) = gyroReading_(:,2:6) - mean(gyroReading_(1500:4000,2:6));
gyr = gyroReading_(1500:4000,2:4) .* pi / 180;
acc = gyroReading_(1500:4000,5:7);

Fs = 50; 
N = length(acc);

%% Print data
ch_plot_imu('time', 1:N, 'acc', acc, 'gyr', gyr);

%% gyro is rad, Acc is m/s^(2)
% gyr = deg2rad(gyr);
% acc = acc*9.795;

% Init
p = zeros(3, 1);
v = zeros(3, 1);
q= [1 0 0 0]';


for i=1:N
    [p ,v , q] = ch_nav_equ_local_tan(p, v, q, acc(i,:)', gyr(i,:)', 1 / Fs, [0, 0, 9.7978]');
    pos(i,:) = p;
    vel(i,:) = v;
end

%3D position plot
figure;
plot3(pos(1,1), pos(1,2), pos(1,3), '-ks');
hold on;
plot3(pos(:,1), pos(:,2), pos(:,3), '.b');
axis equal
xlabel('X(m)');  ylabel('Y(m)');   zlabel('Z(m)'); 
title('3D position');
legend('Start', '3D');

figure(4)
subplot(3,1,1)
plot(vel(:,1));
title('velocity in static');
legend('vx');
subplot(3,1,2)
plot(vel(:,2));
legend('vy');
subplot(3,1,3)
plot(vel(:,3));
legend('vz');

figure;
plot(pos(:,1), pos(:,2), '.b');
hold on;
plot(pos(1,1), pos(1,2), '-ks');
axis equal
title('2D position');
xlabel('X(m)');  ylabel('Y(m)'); 

    
fprintf("Totally %d data，time: %.3fs\n", N, N/Fs);
fprintf("Start point: %.3f %.3f, end point%.3f %.3f, diff:%.3fm\n", pos(1,1), pos(1,2), pos(N,1), pos(N,2), norm(pos(N,:) - pos(1,:)));

