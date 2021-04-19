function [est_C_b_e_new,est_v_eb_e_new,est_r_eb_e_new,est_IMU_bias_new,...
    est_clock_new,P_matrix_new,R_matrix] = TC_KF_Epoch(GNSS_measurements,...
    no_meas,tor_s,est_C_b_e_old,est_v_eb_e_old,est_r_eb_e_old,...
    est_IMU_bias_old,est_clock_old,P_matrix_old,meas_f_ib_b,...
    TC_KF_config,L_ba_b,meas_omega_ib_b,Clock_Reset_Flag,DoubleGNSSHeadingEpoch)
%TC_KF_Epoch - Implements one cycle of the tightly coupled INS/GNSS
% extended Kalman filter plus closed-loop correction of all inertial states
%s
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 12/4/2012 by Paul Groves
% Inputs:
%   GNSS_measurements     GNSS measurement data:
%     Column 1              Pseudo-range measurements (m)
%     Column 2              Pseudo-range rate measurements (m/s)
%     Columns 3-5           Satellite ECEF position (m)
%     Columns 6-8           Satellite ECEF velocity (m/s)
%     Columns 9             Elevation angle (deg)
%   no_meas               Number of satellites for which measurements are
%                         supplied
%   tor_s                 propagation interval (s)
%   est_C_b_e_old         prior estimated body to ECEF coordinate
%                         transformation matrix
%   est_v_eb_e_old        prior estimated ECEF user velocity (m/s)
%   est_r_eb_e_old        prior estimated ECEF user position (m)
%   est_IMU_bias_old      prior estimated IMU biases (body axes)
%   est_clock_old         prior Kalman filter state estimates
%   P_matrix_old          previous Kalman filter error covariance matrix
%   meas_f_ib_b           measured specific force
%   est_L_b_old           previous latitude solution
%   TC_KF_config
%     .gyro_noise_PSD     Gyro noise PSD (rad^2/s)
%     .accel_noise_PSD    Accelerometer noise PSD (m^2 s^-3)
%     .accel_bias_PSD     Accelerometer bias random walk PSD (m^2 s^-5)
%     .gyro_bias_PSD      Gyro bias random walk PSD (rad^2 s^-3)
%     .clock_freq_PSD     Receiver clock frequency-drift PSD (m^2/s^3)
%     .clock_phase_PSD    Receiver clock phase-drift PSD (m^2/s)
%     .pseudo_range_SD    Pseudo-range measurement noise SD (m)
%     .range_rate_SD      Pseudo-range rate measurement noise SD (m/s)
%   Clock_Reset_Flag      If it is 1, it means that the GNSS receiver data has been repaired by clock jump, and the clock drift is forced to zero
% Outputs:
%   est_C_b_e_new     updated estimated body to ECEF coordinate
%                      transformation matrix
%   est_v_eb_e_new    updated estimated ECEF user velocity (m/s)
%   est_r_eb_e_new    updated estimated ECEF user position (m)
%   est_IMU_bias_new  updated estimated IMU biases
%     Rows 1-3          estimated accelerometer biases (m/s^2)
%     Rows 4-6          estimated gyro biases (rad/s)
%   est_clock_new     updated Kalman filter state estimates
%     Row 1             estimated receiver clock offset (m)
%     Row 2             estimated receiver clock drift (m/s)
%   P_matrix_new      updated Kalman filter error covariance matrix
%   R_matrix        measurement noise covariance matrix
% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details
% Modified 2017/8 by LiuXiao BUAA benzenemo@buaa.edu.cn % 20170311B104ZXY
% Constants (sone of these could be changed to inputs at a later date)

omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s
R_0 = 6378137; %WGS84 Equatorial radius in meters
e = 0.0818191908425; %WGS84 eccentricity

% Begins

[est_L_b_old,est_lambda_b_old,~,~,est_C_b_n_old] =...
    ECEF_to_NED(est_r_eb_e_old,est_v_eb_e_old,est_C_b_e_old);
% Skew symmetric matrix of Earth rate
Omega_ie = Skew_symmetric([0,0,omega_ie]);

% SYSTEM PROPAGATION PHASE

% 1. Determine transition matrix using (14.50) (first-order approx)
Phi_matrix = eye(17);
Phi_matrix(1:3,1:3) = Phi_matrix(1:3,1:3) - Omega_ie * tor_s;
Phi_matrix(1:3,13:15) = est_C_b_e_old * tor_s;
Phi_matrix(4:6,1:3) = -tor_s * Skew_symmetric(est_C_b_e_old * meas_f_ib_b);
Phi_matrix(4:6,4:6) = Phi_matrix(4:6,4:6) - 2 * Omega_ie * tor_s;
geocentric_radius = R_0 / sqrt(1 - (e * sin(est_L_b_old))^2) *...
    sqrt(cos(est_L_b_old)^2 + (1 - e^2)^2 * sin(est_L_b_old)^2); % from (2.137)
Phi_matrix(4:6,7:9) = -tor_s * 2 * Gravity_ECEF(est_r_eb_e_old) /...
    geocentric_radius * est_r_eb_e_old' / sqrt (est_r_eb_e_old' *...
    est_r_eb_e_old);
Phi_matrix(4:6,10:12) = est_C_b_e_old * tor_s;
Phi_matrix(7:9,4:6) = eye(3) * tor_s;
Phi_matrix(16,17) = tor_s;
% 2. Determine approximate system noise covariance matrix using (14.82)
Q_prime_matrix = zeros(17);
Q_prime_matrix(1:3,1:3) = eye(3) * TC_KF_config.gyro_noise_PSD * tor_s;
Q_prime_matrix(4:6,4:6) = eye(3) * TC_KF_config.accel_noise_PSD * tor_s;
Q_prime_matrix(10:12,10:12) = eye(3) * TC_KF_config.accel_bias_PSD * tor_s;
Q_prime_matrix(13:15,13:15) = eye(3) * TC_KF_config.gyro_bias_PSD * tor_s;
Q_prime_matrix(16,16) = TC_KF_config.clock_phase_PSD * tor_s;
Q_prime_matrix(17,17) = TC_KF_config.clock_freq_PSD * tor_s;

% 3. Propagate state estimates using (3.14) noting that only the clock
% states are non-zero due to closed-loop correction.
x_est_propagated(1:15,1) = 0;
if Clock_Reset_Flag==1%GNSS clock skip repair identification, for the repaired clock, the residual clock error that has not been repaired is 0
    x_est_propagated(16,1)=0;
    x_est_propagated(17,1)=0;
else
    x_est_propagated(16,1) = est_clock_old(1) + est_clock_old(2) * tor_s;
    x_est_propagated(17,1) = est_clock_old(2);
end

% 4. Propagate state estimation error covariance matrix using (3.46)
P_matrix_propagated = Phi_matrix * (P_matrix_old + 0.5 * Q_prime_matrix) *...
    Phi_matrix' + 0.5 * Q_prime_matrix;

% MEASUREMENT UPDATE PHASE

u_as_e_T = zeros(no_meas,3);
pred_meas = zeros(no_meas,2);

% Finding the position and velocity of user antenna
est_r_ea_e_old=est_r_eb_e_old+est_C_b_e_old*L_ba_b;
est_v_ea_e_old=est_v_eb_e_old+est_C_b_e_old*(Skew_symmetric(meas_omega_ib_b)*L_ba_b);%The earth rotation term (14.121) is ignored here

% Loop measurements
for j = 1:no_meas
    %     % Predict approx range
    %     delta_r = GNSS_measurements(j,3:5)' - est_r_ea_e_old;
    %     approx_range = sqrt(delta_r' * delta_r);
    %
    %     % Calculate frame rotation during signal transit time using (8.36)
    %     C_e_I = [1, omega_ie * approx_range / c, 0;...
    %              -omega_ie * approx_range / c, 1, 0;...
    %              0, 0, 1];
    %GNSS_ The position of the satellite in measurements has been considered in the program creatgnssobsforcouple
    % Predict pseudo-range using (9.165)
    delta_r =  GNSS_measurements(j,3:5)' - est_r_ea_e_old;
    range = sqrt(delta_r' * delta_r);
    pred_meas(j,1) = range + x_est_propagated(16);
    
    % Predict line of sight
    u_as_e_T(j,1:3) = delta_r' / range;
    
    % Predict pseudo-range rate using (8.45) ignore Sagnac
    range_rate = u_as_e_T(j,1:3) * (GNSS_measurements(j,6:8)'-est_v_ea_e_old);
    
    pred_meas(j,2) = range_rate + x_est_propagated(17);
    
end % for j

% 5. Set-up measurement matrix using (14.126)
if TC_KF_config.StationarityFlag == 0 && TC_KF_config.DoubleGNSSDataFlag == 0%The static detection result is dynamic and there is no double antenna observation
    H_matrix = zeros((2 * no_meas),17);
elseif TC_KF_config.StationarityFlag == 1 && TC_KF_config.DoubleGNSSDataFlag == 0%��ֹ�����Ϊ��̬����˫���߹۲�
    H_matrix = zeros((2 * no_meas)+1,17);%Add an azimuth gyroscope to observe
elseif TC_KF_config.StationarityFlag == 0 && TC_KF_config.DoubleGNSSDataFlag == 1%The static detection result is static and there is no double antenna observation
    H_matrix = zeros((2 * no_meas)+3,17);%Add a dual antenna heading observation (the rotation of D direction in n system is decomposed into three axes in E system, so it is 3D)
elseif TC_KF_config.StationarityFlag == 1 && TC_KF_config.DoubleGNSSDataFlag == 1%The static detection result is static and has double antenna observation
    H_matrix = zeros((2 * no_meas)+4,17);%Add azimuth gyroscope and dual gnsss antenna
end
H_matrix(1:no_meas,7:9) = u_as_e_T(1:no_meas,1:3);
H_matrix(1:no_meas,16) = ones(no_meas,1);

H_matrix((no_meas + 1):(2 * no_meas),4:6) = u_as_e_T(1:no_meas,1:3);
H_matrix((no_meas + 1):(2 * no_meas),17) = ones(no_meas,1);

if TC_KF_config.StationarityFlag == 1 && TC_KF_config.DoubleGNSSDataFlag == 0%The static detection result is static and there is no double antenna observation
    H_matrix(2 * no_meas+1,15) = -1;%Zero angular rate correction observation equation
elseif TC_KF_config.StationarityFlag == 0 && TC_KF_config.DoubleGNSSDataFlag == 1%The static detection result is dynamic and has two antennas
    H_matrix(2 * no_meas+1:2 * no_meas+3,1:3) = -eye(3);%Dual GNSS direction finding observation equation
elseif TC_KF_config.StationarityFlag == 1 && TC_KF_config.DoubleGNSSDataFlag == 1%The static detection result is static and has double antenna observation
    H_matrix(2 * no_meas+1,15) = -1;%Zero angular rate correction observation equation
    H_matrix(2 * no_meas+2:2 * no_meas+4,1:3) = -eye(3);%Dual GNSS direction finding observation equation
end

% 6. Set-up measurement noise covariance matrix assuming all measurements
% are independent and have equal variance for a given measurement type.
%According to the height angle, R is amplified locally
index_LowEle=GNSS_measurements(:,9)<30;
ElevationGain=ones(no_meas,1);
ElevationGain(index_LowEle)=1./(2*sind(GNSS_measurements(index_LowEle,9)));
if TC_KF_config.StationarityFlag == 0 && TC_KF_config.DoubleGNSSDataFlag == 0%The static detection result is dynamic and there is no double antenna observation
    R_matrix=eye(no_meas*2);
elseif TC_KF_config.StationarityFlag == 1 && TC_KF_config.DoubleGNSSDataFlag == 0%The static detection result is static and there is no double antenna observation
    R_matrix=eye(no_meas*2+1);
elseif TC_KF_config.StationarityFlag == 0 && TC_KF_config.DoubleGNSSDataFlag == 1%The static detection result is dynamic and has two antennas
    R_matrix=eye(no_meas*2+3);
elseif TC_KF_config.StationarityFlag == 1 && TC_KF_config.DoubleGNSSDataFlag == 1%The static detection result is static and has double antenna observation
    R_matrix=eye(no_meas*2+4);
end

R_matrix(1:no_meas,1:no_meas) = diag(ElevationGain)*...
    TC_KF_config.pseudo_range_SD^2;

R_matrix((no_meas + 1):2*no_meas,(no_meas + 1):2*no_meas) =...
    diag(ElevationGain.*ElevationGain)...
    * TC_KF_config.range_rate_SD^2;

if  TC_KF_config.StationarityFlag == 1 && TC_KF_config.DoubleGNSSDataFlag == 0%The static detection result is static and there is no double antenna observation
    R_matrix(2*no_meas+1,2*no_meas+1)=TC_KF_config.ZARU_DGrySD^2;
elseif TC_KF_config.StationarityFlag == 0 && TC_KF_config.DoubleGNSSDataFlag == 1%The static detection result is dynamic and has two antennas
    R_matrix(2*no_meas+1:2*no_meas+3,2*no_meas+1:2*no_meas+3)=eye(3)*DoubleGNSSHeadingEpoch(1,2)^2;
elseif TC_KF_config.StationarityFlag == 1 && TC_KF_config.DoubleGNSSDataFlag == 1%The static detection result is static and has double antenna observation
    R_matrix(2*no_meas+1,2*no_meas+1)=TC_KF_config.ZARU_DGrySD^2;
    R_matrix(2*no_meas+2:2*no_meas+4,2*no_meas+2:2*no_meas+4)=eye(3)*DoubleGNSSHeadingEpoch(1,2)^2;
end
% 7. Calculate Kalman gain using (3.21)
K_matrix = P_matrix_propagated * H_matrix' /(H_matrix *...
    P_matrix_propagated * H_matrix' + R_matrix);

% 8. Formulate measurement innovations using (14.119)
if TC_KF_config.StationarityFlag == 0 && TC_KF_config.DoubleGNSSDataFlag == 0%The static detection result is dynamic and there is no double antenna observation
    delta_z=zeros(no_meas*2,1);
elseif TC_KF_config.StationarityFlag == 1 && TC_KF_config.DoubleGNSSDataFlag == 0%The static detection result is static and there is no double antenna observation
    delta_z=zeros(no_meas*2+1,1);
elseif TC_KF_config.StationarityFlag == 0 && TC_KF_config.DoubleGNSSDataFlag == 1%The static detection result is dynamic and has two antennas
    delta_z=zeros(no_meas*2+3,1);
elseif TC_KF_config.StationarityFlag == 1 && TC_KF_config.DoubleGNSSDataFlag == 1%The static detection result is static and has double antenna observation
    delta_z=zeros(no_meas*2+4,1);
end

delta_z(1:no_meas,1) = GNSS_measurements(1:no_meas,1) -...
    pred_meas(1:no_meas,1);
delta_z((no_meas + 1):(2 * no_meas),1) = GNSS_measurements(1:no_meas,2) -...
    pred_meas(1:no_meas,2);
if TC_KF_config.StationarityFlag == 1 && TC_KF_config.DoubleGNSSDataFlag == 0%The static detection result is static and there is no double antenna observation
    delta_z(2*no_meas + 1,1)=-meas_omega_ib_b(3);%Zero angular rate correction (Zaru) gyroscope output as innovation
elseif TC_KF_config.StationarityFlag == 0 && TC_KF_config.DoubleGNSSDataFlag == 1%The static detection result is dynamic and has two antennas
    old_Euler=CTM_to_Euler(est_C_b_n_old');
    delta_heading_n=DoubleGNSSHeadingEpoch(1,1)-TC_KF_config.HeadingBias-old_Euler(3);%The course measured by two antennas is subtracted from the course calculated by recursion
    delta_z(no_meas*2+1:no_meas*2+3,1)=HeadingNToEulerE(delta_heading_n,est_L_b_old,est_lambda_b_old);
elseif TC_KF_config.StationarityFlag == 1 && TC_KF_config.DoubleGNSSDataFlag == 1%The static detection result is static and has double antenna observation
    delta_z(2*no_meas + 1,1)=-meas_omega_ib_b(3);%Zero angular rate correction (Zaru) gyroscope output as innovation
    old_Euler=CTM_to_Euler(est_C_b_n_old');
    delta_heading_n=DoubleGNSSHeadingEpoch(1,1)-TC_KF_config.HeadingBias-old_Euler(3);%The course measured by two antennas is subtracted from the course calculated by recursion
    delta_z(no_meas*2+2:no_meas*2+4,1)=HeadingNToEulerE(delta_heading_n,est_L_b_old,est_lambda_b_old);
end

% 9. Update state estimates using (3.24)
x_est_new = x_est_propagated + K_matrix * delta_z;

% 10. Update state estimation error covariance matrix using (3.25)
P_matrix_new = (eye(17) - K_matrix * H_matrix) * P_matrix_propagated;

% CLOSED-LOOP CORRECTION

% Correct attitude, velocity, and position using (14.7-9)
est_C_b_e_new = (eye(3) - Skew_symmetric(x_est_new(1:3))) * est_C_b_e_old;
est_v_eb_e_new = est_v_eb_e_old - x_est_new(4:6);
est_r_eb_e_new = est_r_eb_e_old - x_est_new(7:9);

% Update IMU bias and GNSS receiver clock estimates
est_IMU_bias_new = est_IMU_bias_old + x_est_new(10:15);
est_clock_new = x_est_new(16:17)';

% Ends