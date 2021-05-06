function plotEuler(IMU)
addpath('D:\hfss model\MatlabCode\Tightly coupling program\iXR_GNSS-IMU_TightlyCouplingProgram-master\1_ReadIMU\Mechanization\lib');
addpath('D:\hfss model\MatlabCode\Tightly coupling program\iXR_GNSS-IMU_TightlyCouplingProgram-master\1_ReadIMU\Mechanization\lib\rotation');
%% 真实数据姿�?�纯积分

gyroReading = (IMU(:,5:7) );
gyroReading(:,3) = -gyroReading(:,3);
dt = 0.02; %姿�?�更新周�?: 0.01s = 100Hz
N = length(gyroReading); % 总数据量

eul = zeros(N, 3);

%初始姿�??
Qb2n = [1 0 0 0]';


for i = 1:N
    %theta = deg2rad(gyroReading(i,:)')*dt; %定轴转动�?: 等效旋转矢量 = 角增�? = 角�?�度*dt
    theta = gyroReading(i,:)'*dt;
    Q_m2m_1 = ch_rv2q(theta);
    
    Qb2n  = ch_qmul(Qb2n, Q_m2m_1);
    
    %四元数单位化
    Qb2n = ch_qnormlz(Qb2n);

    %记录每一步的欧拉�?
    eul(i,:) = rad2deg(ch_q2eul(Qb2n));
end

eul(:,3) = eul(:,3);
plot(eul)
legend("PITCH(deg)", "ROLL(deg)", "YAW(deg)");

tmp =eul(end,:);
fprintf("�?终欧拉角: pitch:%.4f° roll:%.4f° yaw:%.4f°\n", tmp(1), tmp(2), tmp(3));
end
