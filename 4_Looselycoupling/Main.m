clc,clear;
%% init
deg_to_rad = pi/180;
rad_to_deg = 1/deg_to_rad;
c = 299792458;
micro_g_to_meters_per_second_squared = 9.80665E-6;
% Interval between GNSS epochs (s)
GNSS_config.epoch_interval = 0.2;

% Initial attitude uncertainty per axis (deg, converted to rad)
TC_KF_config.init_att_unc = deg2rad(5);
% Initial velocity uncertainty per axis (m/s)
TC_KF_config.init_vel_unc = 0.1;
% Initial position uncertainty per axis (m)
TC_KF_config.init_pos_unc = 0.1;
% Initial accelerometer bias uncertainty per instrument (micro-g, converted to m/s^2)
TC_KF_config.init_b_a_unc = 1000 * micro_g_to_meters_per_second_squared;
% Initial gyro bias uncertainty per instrument (deg/hour, converted to rad/sec)
TC_KF_config.init_b_g_unc = 10 * deg_to_rad / 3600;

% NOTE: A large noise PSD is modeled to account for the scale-factor and
% cross-coupling errors that are not directly included in the Kalman filter model
% Gyro noise PSD (deg^2 per hour, converted to rad^2/s)                
TC_KF_config.gyro_noise_PSD = 1e-8;
% Accelerometer noise PSD (micro-g^2 per Hz, converted to m^2 s^-3)                
TC_KF_config.accel_noise_PSD = 3e-4;

% Accelerometer bias random walk PSD (m32 s^-5)
TC_KF_config.accel_bias_PSD = 1E-3;
% Gyro bias random walk PSD (rad^2 s^-3)
TC_KF_config.gyro_bias_PSD = 4E-14;
% GNSS pos measurement noise SD (m)
TC_KF_config.pos_meas_SD =0.1;
% GNSS vel measurement noise SD (m/s)
TC_KF_config.vel_meas_SD = 1e-3;

% Car speed measurement noise SD (m/s)
TC_KF_config.CarSpeed = 1;

TC_KF_config.RecClockPreprocOptions=0;% The default value is 0,If the receiver clock jitter, the value is 2
TC_KF_config.KFMethod='ClassicKF';%ClassicKF means using the classic Kalman filter,M-LSKFClassicKF means using the classic Kalman filter,，IAE-KF表示使用新息对R阵进行膨胀
TC_KF_config.StationarityDetectionInterval=1;
TC_KF_config.StationarityDetectionFlag=[0,0,0,0];%ZUPT [Horizontal velocity threshold, accelerometer measurement standard deviation, frequency domain filter, azimuth rate]
if TC_KF_config.StationarityDetectionFlag(1)==1
    TC_KF_config.StationarityDetectionHorizontalSpeed=0.5;%Horizontal velocity threshold of static detection m/s
    TC_KF_config.StationarityDetectionHSTimeWindow=10;%Time window of horizontal speed detection，s
end
%Zero angular rate correction for observation noise of downward gyro observations SD（rad/s）
TC_KF_config.ZARU_DGrySD=0.00050;
%Observation configuration of dual GNSS antennas
TC_KF_config.DoubleGNSSFlag=0;%1means has double antennas
%% Provide initial position, velocity and attitude
% Data for 2020/11/05
old_time=364534;%20201105 alignment time
old_est_r_ea_e=[-3961765.2144;   3349009.2996;   3698311.6645];%antenna position
old_est_v_ea_e=[0.008125;	-0.018983;	0.002105];
%old_est_v_ea_e=[0.002236;0.034801;0.002225];%receiver velocity
est_clock=[-0.007080535752941*c, 3.487476978027949e+02];
%init attitude roll, pitch, yaw convert to yaw、pitch、roll
attitude_ini = [0;0;-0.7696];
% attitude_ini = [0;0;-2.326737];
L_ba_b=[0;0;-0.6];
%% Data for 2021/04/12
%  old_time = 111184;
%  old_est_r_ea_e = [-3961764.9058;3349009.4333;3698311.9198];
%  old_est_v_ea_e=[0.009361	;0.041011	;0.050424];
%  attitude_ini = [0;0;-0.7520947078188291];
%  L_ba_b=[0;0;-0.6];
 
FilePath.GNSSFile= 'rtk1105_v2.csv';
FilePath.INSFile= 'imu1105_carSpeed.csv';

old_est_C_b_n=Euler_to_CTM(attitude_ini)';%Coordinate transformation matrix from IMU  to local horizontal coordinate system
[old_est_L_a,old_est_lambda_a,old_est_h_a,old_est_v_ea_n] =...
    pv_ECEF_to_NED(old_est_r_ea_e,old_est_v_ea_e);
[~,~,old_est_C_b_e] = NED_to_ECEF(old_est_L_a,...
    old_est_lambda_a,old_est_h_a,old_est_v_ea_n,old_est_C_b_n);
old_est_r_eb_e=old_est_r_ea_e-old_est_C_b_e*L_ba_b;
old_est_v_eb_e=old_est_v_ea_e;%The lever arm effect of velocity is ignored here
Total_GNSS_epoch = 9435 - 483;

% Tightly coupled ECEF Inertial navigation and GNSS integrated navigation
[out_profile,out_IMU_bias_est,out_KF_SD,...
    out_MeasurementNoise_SD,out_Resi,InnovationRes,StdInnovationRes] =...
    Loosly_coupled_INS_GNSS(FilePath,old_time,old_est_r_eb_e,old_est_v_eb_e,attitude_ini,GNSS_config,TC_KF_config,L_ba_b,Total_GNSS_epoch);
%% Plot
% plotCoor(out_profile(:,2:4));
% hold on
% load(FilePath.GNSSFile);
% plotCoor(GNSSTCData(1:length(GNSSTCData)/2-0.5,3:5));

% ubox = csvread('ecefposFromUbox2.csv');
% plotCoor(ubox(1:length(GNSSTCData)/2-0.5,2:4));
% hold on
figure(3)
load('RTK1105.mat');
plot(GNSSTCData(:,3),GNSSTCData(:,4));
hold on
plot(out_profile(1:88418,2),out_profile(1:88418,3));