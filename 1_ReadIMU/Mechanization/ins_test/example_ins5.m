clear;
clc;
close all;
format long
addpath(genpath('..\lib'));
load filter.mat
%% Read IMU
% data = csvread("UranusData.csv", 1, 1);
% acc = data(:,2:4);
% gyr = data(:,5:7);
% mag = data(:,8:10);
data = csvread("Lfet90Right90.csv");
acc = data(1:1000,5:7);
gyr = data(1:1000,2:4);
% gyr = data(3000:6800,5:7) - mean(data(1:3000,5:7));
% acc = data(5400:6800,2:4)- mean(data(1:3000,2:4));
% acc(:,3) = acc(:,3) + 9.7978;
% acc= gyr;
% acc(:,1) = data(3000:6800,9) ;
% acc(:,2) = 0;
% acc(:,3) = 9.7978;
%mag = data(:,8:10);

% acc = data(:,8:10);
% gyr = data(:,11:13);

q= [1 0 0 0]';
% q   = data(1,4:7)';

Fs = 50; 
dt = 1 / Fs;
N = length(acc);

fprintf("Totally %d data，time:%.3fs\n", N, N/Fs);

% gyr = deg2rad(gyr);
% acc = acc*9.8;

% Init
p = zeros(3, 1);
v = zeros(3, 1);

for i=1:N
    [p ,v , q] = ch_nav_equ_local_tan(p, v, q, acc(i,:)', gyr(i,:)', 1 / Fs, [0, 0, -9.7978]');
 
    linAcc(i,:) = ch_qmulv(q, acc(i,:));
    linAcc(i,3) = linAcc(i,3) -9.8;
    R(:,:,i) = ch_q2m(q);
end

% Velocity
linVel = zeros(size(linAcc));
for i = 2:length(linAcc)
    linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * dt;
end

order = 1;
filtCutOff = 0.1;
%[b, a] = butter(order, (2*filtCutOff)/(1/dt), 'high');
%linVelHP = filtfilt(b, a, linVel);
linVelHP = filter(Hd, linVel);

% Position
linPos = zeros(size(linVelHP));
for i = 2:length(linVelHP)
    linPos(i,:) = linPos(i-1,:) + linVelHP(i,:) * dt;
end

%[b, a] = butter(order, (2*filtCutOff)/(1/dt), 'high');
%linPosHP = filtfilt(b, a, linPos);
linPosHP = filter(Hd, linPos);


%% Plot
figure('NumberTitle', 'off', 'Name', ' velocity');
subplot(2,1,1);
plot(linVel);
title(' velocity');
legend('X', 'Y', 'Z');
subplot(2,1,2);
plot(linVelHP);
title('HP velocity');
legend('X', 'Y', 'Z');



figure('NumberTitle', 'off', 'Name', ' position');
subplot(2,1,1);
plot(linPos);
title(' position');
legend('X', 'Y', 'Z');
subplot(2,1,2);
plot(linPosHP);
title('HP position');
legend('X', 'Y', 'Z');


figure('NumberTitle', 'off', 'Name', 'raw data');
subplot(2,1,1);
plot(acc);
legend("X", "Y", "Z");
title("Acc");
subplot(2,1,2);
plot(gyr);
title("Gyro");
legend("X", "Y", "Z");

% 3D position
%linPosHP = mag;

figure('NumberTitle', 'off', 'Name', '3D position');
plot3(linPosHP(1,1), linPosHP(1,2), linPosHP(1,3), '-ks');
hold on;
plot3(linPosHP(:,1), linPosHP(:,2), linPosHP(:,3), '.b');
axis equal
xlabel('X(m)');  ylabel('Y(m)');   zlabel('Z(m)'); 
title('3D position');
legend('Start', '3D');





% SamplePlotFreq = 4;
% 
% SixDOFanimation(linPosHP, R, ...
%                 'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
%                 'Position', [9 39 400 400], ...
%                 'AxisLength', 0.1, 'ShowArrowHead', false, ...
%                 'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
%                 'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/dt) / SamplePlotFreq));            
