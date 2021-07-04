function plotEuler(IMU,heading)
addpath('F:\ExperienceData\IMUPaper\nav_matlab-master (1)\nav_matlab-master\lib');
addpath('F:\ExperienceData\IMUPaper\nav_matlab-master (1)\nav_matlab-master\lib\rotation');
%% 真实数据姿态纯积分

gyroReading = -(IMU(:,5:7) );
gyroReading(:,3) = -gyroReading(:,3);
dt = 0.02; %姿态更新周期: 0.01s = 100Hz
N = length(gyroReading); % 总数据量

eul = zeros(N, 3);

%初始姿态
Qb2n = [1 0 0 0]';


for i = 1:N
    %theta = deg2rad(gyroReading(i,:)')*dt; %定轴转动下: 等效旋转矢量 = 角增量 = 角速度*dt
    theta = gyroReading(i,:)'*dt;
    Q_m2m_1 = ch_rv2q(theta);
    
    Qb2n  = ch_qmul(Qb2n, Q_m2m_1);
    
    %四元数单位化
    Qb2n = ch_qnormlz(Qb2n);

    %记录每一步的欧拉角
    eul(i,:) = rad2deg(ch_q2eul(Qb2n));
end


plot(IMU(:,1),eul+heading);
legend("PITCH(deg)", "ROLL(deg)", "YAW(deg)");

tmp =eul(end,:);
fprintf("最终欧拉角: pitch:%.4f° roll:%.4f° yaw:%.4f°\n", tmp(1), tmp(2), tmp(3));
end
