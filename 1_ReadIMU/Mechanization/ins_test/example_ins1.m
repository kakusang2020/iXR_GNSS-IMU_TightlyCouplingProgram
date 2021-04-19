%% INS 
clear;
clc;
close all;
addpath('..\lib');
%%
Fs = 100;
N = Fs*10;
dataimu = csvread('imu1105.csv');
% gyr = [0.1, 0.2, 0.3];
% acc = [0, 0, 1];
gyr = dataimu(1:7000,5:7);
acc = dataimu(1:7000,2:4);
%acc = acc * 9.795;
%gyr = deg2rad(gyr);

% Mechanization
p = zeros(3, 1);
v = zeros(3, 1);
q= [1 0 0 0]';

for i=1:N
    [p ,v, q] = ch_nav_equ_local_tan(p, v, q, acc(N,:)', gyr(N,:)' , 1 / Fs, [0, 0, -9.588903678912498]');
    pos(i,:) = p;
    eul(i,:) = rad2deg(ch_q2eul(q));
end


plot(pos);
title("Position Result");
legend("X", "Y", "Z");

figure;
plot(eul);
title("Euler");
legend("P", "R", "Y");

fprintf('Pure integral test: Gyro bias(rad):%.3f %.3f %.3f\n', gyr(1), gyr(2), gyr(3));
fprintf('Pure integral test: Acc bias(m/s^(2)):%.3f %.3f %.3f\n', acc(1), acc(2), acc(3));

fprintf('Result:%dtimes time:%.3fs\n', N, N /Fs);
fprintf('Error(m): %.3f %.3f %.3f\n', pos(N, 1),  pos(N, 2),  pos(N, 3));


