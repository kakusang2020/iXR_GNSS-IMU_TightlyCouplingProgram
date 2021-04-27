clc,clear
load('IMUData.mat');
%plotEuler(IMUData);
IMUData(:,2:7) = IMUData(:,2:7) - mean(IMUData(1:3000,2:7));
IMUData(:,4) = IMUData(:,4) + 9.7978;
old_r_eb_e=[0;0;0];
old_v_eb_e=[0;0;0];
old_C_b_e = eye(3);
pos = [];
vel = [];
for i = 2:3000
    tor_i = (IMUData(i,1) - IMUData(i-1,1));
    f_ib_b = IMUData(i,2:4);
    omega_ib_b = IMUData(i,5:7);
    [old_r_eb_e,old_v_eb_e,old_C_b_e] = OnlyIMU(tor_i,old_r_eb_e,...
        old_v_eb_e,old_C_b_e,f_ib_b',omega_ib_b');
    pos = [pos;old_r_eb_e'];
    vel = [vel;old_v_eb_e'];
end
%% 3D position plot
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