function [est_C_b_e_new,est_v_eb_e_new,est_r_eb_e_new,est_IMU_bias_new,...
    P_NED_matrix_new] = LC_LIMIT_KF_Epoch(tor_s,...
    est_C_b_e_old,est_v_eb_e_old,est_r_eb_e_old,est_IMU_bias_old,...
    P_NED_matrix_old,meas_f_ib_b,meas_omega_ib_b,LC_KF_config)
%LC_KF_Epoch - Implements one cycle of the loosely coupled INS/GNSS
% Kalman filter plus closed-loop correction of all inertial states
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 12/4/2012 by Paul Groves
%
% Inputs:
%   GNSS_r_eb_e           GNSS estimated ECEF user position (m)
%   GNSS_v_eb_e           GNSS estimated ECEF user velocity (m/s)
%   tor_s                 propagation interval (s)
%   est_C_b_e_old         prior estimated body to ECEF coordinate
%                         transformation matrix
%   est_v_eb_e_old        prior estimated ECEF user velocity (m/s)
%   est_r_eb_e_old        prior estimated ECEF user position (m)
%   est_IMU_bias_old      prior estimated IMU biases (body axes)
%   P_matrix_old          previous Kalman filter error covariance matrix
%   meas_f_ib_b           measured specific force
%   est_L_b_old           previous latitude solution
%   LC_KF_config
%     .gyro_noise_PSD     Gyro noise PSD (rad^2/s)
%     .accel_noise_PSD    Accelerometer noise PSD (m^2 s^-3)
%     .accel_bias_PSD     Accelerometer bias random walk PSD (m^2 s^-5)
%     .gyro_bias_PSD      Gyro bias random walk PSD (rad^2 s^-3)
%     .pos_meas_SD            Position measurement noise SD per axis (m)
%     .vel_meas_SD            Velocity measurement noise SD per axis (m/s)
%
% Outputs:
%   est_C_b_e_new     updated estimated body to ECEF coordinate
%                      transformation matrix
%   est_v_eb_e_new    updated estimated ECEF user velocity (m/s)
%   est_r_eb_e_new    updated estimated ECEF user position (m)
%   est_IMU_bias_new  updated estimated IMU biases
%     Rows 1-3          estimated accelerometer biases (m/s^2)
%     Rows 4-6          estimated gyro biases (rad/s)
%   P_matrix_new      updated Kalman filter error covariance matrix

% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details

% Constants (sone of these could be changed to inputs at a later date)
c = 299792458; % Speed of light in m/s
omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s
R_0 = 6378137; %WGS84 Equatorial radius in meters
e = 0.0818191908425; %WGS84 eccentricity
STA.STA(1).Coor(1:3) = [-3961904.939,3348993.763,3698211.764];
% Begins

% Skew symmetric matrix of Earth rate
Omega_ie = Skew_symmetric([0,0,omega_ie]);

% SYSTEM PROPAGATION PHASE
%% Prepare IMU pos&vel from EFEF2NED
[L_b_old,lambda_b_old,h_b_old,v_eb_n_old,C_b_n_old] = ECEF_to_NED(est_r_eb_e_old,est_v_eb_e_old,est_C_b_e_old);
Rotation=[   -sin(L_b_old)*cos(lambda_b_old),-sin(L_b_old)*sin(lambda_b_old),cos(L_b_old);
    -sin(lambda_b_old),cos(lambda_b_old),0;
    cos(L_b_old)*cos(lambda_b_old),cos(L_b_old)*sin(lambda_b_old),sin(L_b_old)];
NEU=Rotation*(est_r_eb_e_old-STA.STA(1).Coor(1:3))';
NED=[NEU(1);NEU(2);-NEU(3)];
%% 1. Determine transition matrix using (14.50) (first-order approx)
P_NED_matrix = eye(15);
% Phi_matrix(1:3,1:3) = Phi_matrix(1:3,1:3) - Omega_ie * tor_s;
P_NED_matrix(1:3,13:15) = C_b_n_old * tor_s;
P_NED_matrix(4:6,1:3) = -tor_s * Skew_symmetric(C_b_n_old * meas_f_ib_b);
% geocentric_radius = R_0 / sqrt(1 - (e * sin(est_L_b_old))^2) *...
%     sqrt(cos(est_L_b_old)^2 + (1 - e^2)^2 * sin(est_L_b_old)^2); % from (2.137)
% Phi_matrix(4:6,7:9) = -tor_s * 2 * Gravity_ECEF(est_r_eb_e_old) /...
%     geocentric_radius * est_r_eb_e_old' / sqrt (est_r_eb_e_old' *...
%     est_r_eb_e_old);
P_NED_matrix(4:6,10:12) = C_b_n_old * tor_s;
P_NED_matrix(7:9,4:6) = eye(3) * tor_s;

% 2. Determine approximate system noise covariance matrix using (14.82)
Q_prime_matrix = zeros(15);
Q_prime_matrix(1:3,1:3) = eye(3) * LC_KF_config.gyro_noise_PSD * tor_s;
Q_prime_matrix(4:6,4:6) = eye(3) * LC_KF_config.accel_noise_PSD * tor_s;
Q_prime_matrix(10:12,10:12) = eye(3) * LC_KF_config.accel_bias_PSD * tor_s;
Q_prime_matrix(13:15,13:15) = eye(3) * LC_KF_config.gyro_bias_PSD * tor_s;

% 3. Propagate state estimates using (3.14) noting that all states are zero
% due to closed-loop correction.
x_est_propagated(1:15,1) = 0;

% 4. Propagate state estimation error covariance matrix using (3.46)
P_matrix_propagated = P_NED_matrix * (P_NED_matrix_old + 0.5 * Q_prime_matrix) *...
    P_NED_matrix' + 0.5 * Q_prime_matrix;

% MEASUREMENT UPDATE PHASE

% 5. Set-up measurement matrix using (14.115)
H_matrix = zeros(1,15);
H_matrix(6) = -eye(1);
% 6. Set-up measurement noise covariance matrix assuming all components of
% GNSS position and velocity are independent and have equal variance.
% R_matrix(1:3,1:3) = eye(3) * LC_KF_config_CarSpeed^2 * 1;
R_matrix(1:1,1:1) = eye(1) * 0.001;
% 7. Calculate Kalman gain using (3.21)
K_matrix = P_matrix_propagated * H_matrix' * inv(H_matrix *...
    P_matrix_propagated * H_matrix' + R_matrix);

% 8. Formulate measurement innovations using (14.102), noting that zero
% lever arm is assumed here
delta_z(1:1,1) = 0 -v_eb_n_old(3);

% 9. Update state estimates using (3.24)
x_est_new = x_est_propagated + K_matrix * delta_z;

% 10. Update state estimation error covariance matrix using (3.25)
P_NED_matrix_new = (eye(15) - K_matrix * H_matrix) * P_matrix_propagated;

% CLOSED-LOOP CORRECTION
%% Prepare IMU pos&vel from NED2ECEF
%  [r_eb_e,v_eb_e,C_b_e] = NED_to_ECEF(L_b,lambda_b,h_b,v_eb_n,C_b_n);
% Correct attitude, velocity, and position using (14.7-9)
est_C_b_n_new = (eye(3) - Skew_symmetric(x_est_new(1:3))) * C_b_n_old;
est_v_eb_n_new = v_eb_n_old - x_est_new(4:6);
est_r_eb_n_new = NED - x_est_new(7:9);
est_r_eb_e_new = Rotation' * [est_r_eb_n_new(1);est_r_eb_n_new(2);-est_r_eb_n_new(3)] + STA.STA(1).Coor(1:3)';
[~,est_v_eb_e_new,est_C_b_e_new] = NED_to_ECEF(L_b_old,lambda_b_old,h_b_old,est_v_eb_n_new,est_C_b_n_new);
% Update IMU bias estimates
est_IMU_bias_new = est_IMU_bias_old + x_est_new(10:15);

% % Constants (sone of these could be changed to inputs at a later date)
% c = 299792458; % Speed of light in m/s
% omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s
% R_0 = 6378137; %WGS84 Equatorial radius in meters
% e = 0.0818191908425; %WGS84 eccentricity
% STA.STA(1).Coor(1:3) = [-3961904.939,3348993.763,3698211.764];
% % Begins
% 
% % Skew symmetric matrix of Earth rate
% Omega_ie = Skew_symmetric([0,0,omega_ie]);
% 
% % SYSTEM PROPAGATION PHASE
% %% Prepare IMU pos&vel from EFEF2NED
% [L_b_old,lambda_b_old,h_b_old,v_eb_n_old,C_b_n_old] = ECEF_to_NED(est_r_eb_e_old,est_v_eb_e_old,est_C_b_e_old);
% Rotation=[   -sin(L_b_old)*cos(lambda_b_old),-sin(L_b_old)*sin(lambda_b_old),cos(L_b_old);
%     -sin(lambda_b_old),cos(lambda_b_old),0;
%     cos(L_b_old)*cos(lambda_b_old),cos(L_b_old)*sin(lambda_b_old),sin(L_b_old)];
% NEU=Rotation*(est_r_eb_e_old-STA.STA(1).Coor(1:3))';
% NED=[NEU(1);NEU(2);-NEU(3)];
% 
% settings = gnss_imu_local_tan_example_settings();
% x=[NED;v_eb_n_old;ch_m2q(C_b_n_old)];
% u_h=[meas_f_ib_b';meas_omega_ib_b'];
% [F, G] = state_space_model(x, u_h, tor_s);            
%             P_NED_matrix_new = F*P_NED_matrix_old*F' + G*Q*G';     
%         % 量测更新         
%                 y = [CarSpeed_,0,0];
%                 H = [zeros(3,3) eye(3) zeros(3,9)];
%                 R = [settings.sigma_gps^2*eye(3)];
%                 
%                 % Calculate the Kalman filter gain.
%                 K=(P_NED_matrix_old*H')/(H*P_NED_matrix_old*H'+R);
%                 
%                 z = [zeros(9,1); delta_u] + K*(y - x(4:6));
%                 
%                 %% Correct the navigation states using current perturbation estimates.
%                 
%                 % 位置速度反馈
%                 x(1:6) = x(1:6) + z(1:6);
%                 
%                 % 失准角反馈到姿态
%                 q = x(7:10);
%                 q = ch_qmul(ch_rv2q(z(7:9)), q);
%                 x(7:10) = q;
%                 
%                 delta_u = z(10:15);
%                 
%                 %更新P 使用Joseph 形式，取代 (I-KH)*P, 这么数值运算更稳定
%                 I_KH = (eye(size(P_NED_matrix_new,1))-K*H);
%                 P_NED_matrix_new= I_KH*P_NED_matrix_new*I_KH' + K*R*K';