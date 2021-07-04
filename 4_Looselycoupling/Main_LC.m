clear;
addpath('../Lib/GNSS_Tool_Lib');
addpath('../Lib/INS_Tool_Lib');
%% init
deg_to_rad = pi/180;
rad_to_deg = 1/deg_to_rad;
c = 299792458;
micro_g_to_meters_per_second_squared = 9.80665E-6;

% Initial attitude uncertainty per axis (deg, converted to rad)
LC_KF_config.init_att_unc = deg2rad(5);
% Initial velocity uncertainty per axis (m/s)
LC_KF_config.init_vel_unc = 0.1;
% Initial position uncertainty per axis (m)
LC_KF_config.init_pos_unc = 0.1;
% Initial accelerometer bias uncertainty per instrument (micro-g, converted to m/s^2)
LC_KF_config.init_b_a_unc = 1000 * micro_g_to_meters_per_second_squared;
% Initial gyro bias uncertainty per instrument (deg/hour, converted to rad/sec)
LC_KF_config.init_b_g_unc = 10 * deg_to_rad / 3600;
%% NOTE: A large noise PSD is modeled to account for the scale-factor and
% cross-coupling errors that are not directly included in the Kalman filter model
% Gyro noise PSD (deg^2 per hour, converted to rad^2/s)                
LC_KF_config.gyro_noise_PSD = 1.878e-12;
% Accelerometer noise PSD (micro-g^2 per Hz, converted to m^2 s^-3)                
LC_KF_config.accel_noise_PSD = 1e-3;

% Accelerometer bias random walk PSD (m32 s^-5)
LC_KF_config.accel_bias_PSD = 5e-7;
% Gyro bias random walk PSD (rad^2 s^-3)
LC_KF_config.gyro_bias_PSD = 5.7453e-14;%4E-14;
% GNSS pos measurement noise SD (m)
LC_KF_config.pos_meas_SD =0.03;
% GNSS vel measurement noise SD (m/s)
LC_KF_config.vel_meas_SD = 1e-4;

% Car speed measurement noise SD (m/s)
LC_KF_config.CarSpeed = 1;
% Interval between GNSS epochs (s)
GNSS_config.epoch_interval = 0.2;

LC_KF_config.KFMethod='ClassicKF';
LC_KF_config.StationarityDetectionInterval=1;
LC_KF_config.StationarityDetectionFlag=[0,0,0,0];%ZUPT [Horizontal velocity threshold, accelerometer measurement standard deviation, frequency domain filter, azimuth rate]
if LC_KF_config.StationarityDetectionFlag(1)==1
    LC_KF_config.StationarityDetectionHorizontalSpeed=0.5;%Horizontal velocity threshold of static detection m/s
    LC_KF_config.StationarityDetectionHSTimeWindow=10;%Time window of horizontal speed detection£¬s
end
%Zero angular rate correction for observation noise of downward gyro observations SD£¨rad/s£©
LC_KF_config.ZARU_DGrySD=0.00050;
%Observation configuration of dual GNSS antennas
LC_KF_config.DoubleGNSSFlag=0;%1means has double antennas
%% Provide initial position, velocity and attitude
% Data for 2020/11/05
% old_time=364534;%20201105 alignment time
% old_est_r_ea_e=[-3961765.2144;   3349009.2996;   3698311.6645];%antenna position
% old_est_v_ea_e=[0.008125;	-0.018983;	0.002105];
% %old_est_v_ea_e=[0.002236;0.034801;0.002225];%receiver velocity
% est_clock=[-0.007080535752941*c, 3.487476978027949e+02];
% %init attitude roll, pitch, yaw convert to yaw¡¢pitch¡¢roll
% attitude_ini = [0;0;-0.7696];
% % attitude_ini = [0;0;-2.326737];
% L_ba_b=[0;0;-0.6];
% FilePath.GNSSFile= 'rtk1105_v2.csv';
% FilePath.INSFile= 'imu1105_carSpeed.csv';
%% Data for 2021/04/12
%  old_time = 111184;
%  old_est_r_ea_e = [-3961764.9058;3349009.4333;3698311.9198];
%  old_est_v_ea_e=[0.009361	;0.041011	;0.050424];
%  attitude_ini = [0.0057;-0.0373;-0.7520947078188291];%  -0.0057    -0.0373-0.7520947078188291
%  L_ba_b=[0;0.3;0];
%  FilePath.GNSSFile= 'rtk0412_yize.csv';
%  FilePath.INSFile= 'imu0412_epsonG370_BLD.csv';
%  FilePath.GNSSVelFile= '0412velocity_v3.csv';%'GNSSVel0412.csv'; 
 %% Data for 2021/05/21
%  old_time = 453600;
%  old_est_r_ea_e = [-3961765.3215;   3349009.3874;   3698311.4879];
%  old_est_v_ea_e=[0.000	;0.00	;0.00];
%  attitude_ini = [0;0;-(90-46+1.6)/180*pi];
%  L_ba_b=[0;0;0];
%  FilePath.GNSSFile='0521clasECEF.csv';% ''0521RTK_hub.csv';%
%  FilePath.INSFile= '0521imuEstelle.csv';%'0521imuG370.csv';%
%  FilePath.GNSSVelFile= '0521velocity.csv';%'GNSSVel0412.csv';
%% Data for 2021/06/09
 old_time = 284042;
 old_est_r_ea_e = [-3961766.0534;   3349008.9325;   3698311.0212];
 old_est_v_ea_e=[0.000	;0.00	;0.00];
 attitude_ini = [0;0;(-45+180)/180*pi];
 L_ba_b=[-1;0;-0.7];
 FilePath.Route='../Data/20210609';
 FilePath.GNSSFile='0609RTKB.csv';% ''0521RTK_hub.csv';%
 FilePath.INSFile= '0609G370imu.csv';%'0521imuG370.csv';%
 FilePath.GNSSVelFile= '0609GNSSvelocity.csv';%'GNSSVel0412.csv';
%  FilePath.SensorFile='0609Sensor.csv';
 FilePath.uboxFile='0609uboxB.csv'; %For compare
 
old_est_C_b_n=Euler_to_CTM(attitude_ini)';%Coordinate transformation matrix from IMU  to local horizontal coordinate system
[old_est_L_a,old_est_lambda_a,old_est_h_a,old_est_v_ea_n] =...
    pv_ECEF_to_NED(old_est_r_ea_e,old_est_v_ea_e);
[~,~,old_est_C_b_e] = NED_to_ECEF(old_est_L_a,...
    old_est_lambda_a,old_est_h_a,old_est_v_ea_n,old_est_C_b_n);
old_est_r_eb_e=old_est_r_ea_e-old_est_C_b_e*L_ba_b;
old_est_v_eb_e=old_est_v_ea_e;%The lever arm effect of velocity is ignored here
Total_GNSS_epoch = 10000;

% Tightly coupled ECEF Inertial navigation and GNSS integrated navigation
[out_profile,out_IMU_bias_est,out_KF_SD,...
    out_MeasurementNoise_SD,out_Resi,InnovationRes,StdInnovationRes] =...
    Loosly_coupled_INS_GNSS(FilePath,old_time,old_est_r_eb_e,old_est_v_eb_e,attitude_ini,GNSS_config,LC_KF_config,L_ba_b,Total_GNSS_epoch);
%% Plot
figure(3)
RTKData_ = csvread(FilePath.GNSSFile);
plotCoor(RTKData_(1:end,2:4));
hold on
plotCoor(out_profile(1:end,2:4));

figure(4)
ubox = csvread(FilePath.uboxFile);
ENUPos2 = ECEF2ENUPos(ubox);
Compar2file([out_profile(:,1),out_profile(:,11:13)],ENUPos2);