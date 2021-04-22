clear;
clc
close all;
addpath('..\lib');
%% Read IMU
% load gyroReading;
% gyroReading_ = csvread('imu1105.csv');
% gyroReading = gyroReading_(:,5:7) .* pi/180;
gyroReading_ = csvread('g370_0412.csv');
gyroReading = -(gyroReading_(:,2:4) - mean(gyroReading_(1:4000,2:4))) .* pi/180;
% gyroReading = -(gyroReading_(:,2:4) ) .* pi/180;
gyroReading(:,3) = -gyroReading(:,3);
dt = 0.02; %Attitude update cycle: 0.01s = 100Hz
N = length(gyroReading); % Total data volume

eul = zeros(N, 3);

%Initial attitude
Qb2n = [1 0 0 0]';


for i = 1:N
    %theta = deg2rad(gyroReading(i,:)')*dt; %Under the initial attitude fixed axis rotation: equivalent rotation vector = angular increment = angular velocity *dt
    theta = gyroReading(i,:)'*dt;
    Q_m2m_1 = ch_rv2q(theta);
    
    Qb2n  = ch_qmul(Qb2n, Q_m2m_1);
    
    %Quaternion unit
    Qb2n = ch_qnormlz(Qb2n);

    %Euler angle
    eul(i,:) = rad2deg(ch_q2eul(Qb2n));
end


plot(eul)
legend("PITCH(deg)", "ROLL(deg)", "YAW(deg)");

tmp =eul(end,:);
fprintf("Euler angle: pitch:%.4f° roll:%.4f° yaw:%.4f°\n", tmp(1), tmp(2), tmp(3));

