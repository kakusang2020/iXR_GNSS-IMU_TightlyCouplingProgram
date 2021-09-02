clear;
addpath('../Lib/GNSS_Tool_Lib');
addpath('../Lib/INS_Tool_Lib');
%% init
deg_to_rad = pi/180;
rad_to_deg = 1/deg_to_rad;
c = 299792458;
micro_g_to_meters_per_second_squared = 9.80665E-6;
% Interval between GNSS epochs (s)
GNSS_config.epoch_interval = 0.2;
% Mask angle (deg)
GNSS_config.mask_angle = 15;
% Mask Carrier to noise ratio(dbhz)
GNSS_config.mask_SignalStrenth = 35;
% Inner System Bias(m)
% GNSS_config.ISBBDS_GPS = 59.5*c/1e9;%2018206 ClockBDS-ClockGPS
GNSS_config.ISBBDS_GPS =0;%69.13*c/1e9;%2018165 ClockBDS-ClockGPS
% Max Number of satellites
GNSS_config.intend_no_GNSS_meas = 30;%If the number of observations exceeds this value, start satellite screening
% Satellite to Omit
GNSS_config.omit =131:160;%Control the GNSS type involved in the solution through this configuration
% GNSS type and PRN mapping list
GNSS_config.GPSPRNList=1:32;
GNSS_config.QZSSPRNList=33:40;
GNSS_config.GLOPRNList=41:80;
GNSS_config.GLONASSPRNList=131:160;
GNSS_config.BDSPRNList=81:130;

% Initial attitude uncertainty per axis (deg, converted to rad)
TC_KF_config.init_att_unc = deg2rad(20);
% Initial velocity uncertainty per axis (m/s)
TC_KF_config.init_vel_unc = 0.1;
% Initial position uncertainty per axis (m)
TC_KF_config.init_pos_unc = 10;
% Initial accelerometer bias uncertainty per instrument (micro-g, converted
% to m/s^2)
TC_KF_config.init_b_a_unc = 10000 * micro_g_to_meters_per_second_squared;
% Initial gyro bias uncertainty per instrument (deg/hour, converted to rad/sec)
TC_KF_config.init_b_g_unc = 10 * deg_to_rad / 3600;
% Initial clock offset uncertainty per axis (m)
TC_KF_config.init_clock_offset_unc = 10;
% Initial clock drift uncertainty per axis (m/s)
TC_KF_config.init_clock_drift_unc = 0.1;

% Gyro noise PSD (deg^2 per hour, converted to rad^2/s)                
TC_KF_config.gyro_noise_PSD = 1.878e-13;
% Accelerometer noise PSD (micro-g^2 per Hz, converted to m^2 s^-3)                
TC_KF_config.accel_noise_PSD = 1e-3;

% Accelerometer bias random walk PSD (m32 s^-5)
TC_KF_config.accel_bias_PSD = 1e-5;
% Gyro bias random walk PSD (rad^2 s^-3)
TC_KF_config.gyro_bias_PSD = 5.7453e-14;%4E-14;
% Receiver clock frequency-drift PSD (m^2/s^3)
TC_KF_config.clock_phase_PSD = 1;
% Pseudo-range measurement noise SD (m)
TC_KF_config.clock_freq_PSD = 1;
% Receiver clock phase-drift PSD (m^2/s)
TC_KF_config.pseudo_range_SD = 1;
% Pseudo-range rate measurement noise SD (m/s)
TC_KF_config.range_rate_SD = 6e-3;
TC_KF_config.RecClockPreprocOptions=2;% The default value is 0,If the receiver clock jitter, the value is 2
TC_KF_config.KFMethod='M-LSKF';%'M-LSKF';%ClassicKF means using the classic Kalman filter,M-LSKFClassicKF means using the classic Kalman filter,，IAE-KF表示使用新息对R阵进行膨胀
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
%% 20201105
FilePath.Route='../Data/20201105';
FilePath.GNSSFile= 'test2v2.csv';%'test2GQEB.csv';
FilePath.INSFile=  'Estelleimu1105.csv';
 FilePath.uboxFile='ubox1105.csv';
old_time=364633;%20201105 alignment time
old_est_r_ea_e=[-3961765.166;	3349009.269;	3698311.598];%antenna position
old_est_v_ea_e=[0;0;0];
%old_est_v_ea_e=[0.002236;0.034801;0.002225];%receiver velocity
est_clock=[-0.007080535752941*c, 3.487476978027949e+02];
%init attitude roll, pitch, yaw convert to yaw、pitch、roll
attitude_ini = [0;0;-0.7696];
% attitude_ini = [0;0;-2.326737];
L_ba_b=[0;0;0];
%% 20210412
% FilePath.Route='../Data/20210412';
% FilePath.GNSSFile= 'test2v3.csv';
% FilePath.INSFile= 'imu0412_epsonG370_BLD.csv';
%  FilePath.uboxFile='ubox0412.csv';
% old_time = 111184;
%  old_est_r_ea_e = [-3961764.9058;3349009.4333;3698311.9198];
%  old_est_v_ea_e=[0.009361	;0.041011	;0.050424];
%  attitude_ini = [0;0;-0.8080947078188291];%  -0.0057    -0.0373-0.7520947078188291
%  L_ba_b=[0;0.3;0];
%  est_clock = [ 0.005965136750345*c, 3.219616160886362e+02];
%% 20210521
% FilePath.Route='../Data/20210521';
% FilePath.GNSSFile='test20521GQEBR.csv';
% FilePath.INSFile='0521imuG370.csv';
%  old_time = 453600;
%  old_est_r_ea_e = [-3961765.3218;   3349009.3809 ;  3698311.4796];
%  old_est_v_ea_e=[0.000	;0.00	;0.00];
%  attitude_ini = [0;0;-(90-46+1.6)/180*pi];
%  L_ba_b=[0;-0.3;-0.8];
%  est_clock=[891358.2367,133.881512];
%% 20210609
% FilePath.Route='../Data/20210609';
% FilePath.GNSSFile='test2GQEB.csv';
% FilePath.INSFile='0609G370imu.csv';
% FilePath.INSFile='0609G370imu_opensky.csv';
% FilePath.uboxFile='0609uboxB.csv';
%  old_time = 284042;
%  old_est_r_ea_e = [-3961766.0534;   3349008.9325;   3698311.0212];
%  old_est_v_ea_e=[0.000	;0.00	;0.00];
%  attitude_ini = [0;0;(-45)/180*pi];
%  L_ba_b=[-1;0;-0.7];
%  est_clock=[8,376.826855];%928532.17477
  % open sky
%   old_time = 284818;
%  old_est_r_ea_e = [-3963790.91;	3349376.728;	3695826.024];
%  old_est_v_ea_e=[2.1;	5.52;	-2.74];
%  attitude_ini = [0;0;(-121.5)/180*pi];
%   L_ba_b=[-1.2;0.5;-0.7];%[-1.2;0.2;-0.7];
%   est_clock=[8,376.826855];%928532.17477
%% 2021/03/02
% 1st
%  old_time = 174205;
%  old_est_r_ea_e = [-3961766.643;	3349009.28;	3698310.787];
%  old_est_v_ea_e=[0.000	;0.00	;0.00];
%  attitude_ini = [0;0;(-43)/180*pi];
%  est_clock=[928532.17477,376.826855];
%  L_ba_b=[0;0;-0.7];
%  FilePath.Route='../Data/20210302/1st';
%  FilePath.INSFile= 'Estelleimu.csv';%'0521imuG370.csv';%
%  FilePath.GNSSFile= 'test203021.csv';%'GNSSVel0412.csv';
%  FilePath.uboxFile='ubox03021.csv'; %For compare

% % 3th
% FilePath.Route='../Data/20210302/3th';
% FilePath.GNSSFile= 'test2GQEBR.csv';%'test203023.csv';
% FilePath.INSFile= 'Estelleimu03023.csv';
% FilePath.uboxFile='poslv03023.csv';
%   old_time = 187540;
%  old_est_r_ea_e = [-3961764.941;	3349009.756;	3698312.325];
%  old_est_v_ea_e=[0.000	;0.00	;0.00];
%  attitude_ini = [0;0;(-49.5)/180*pi];
%  L_ba_b=[0;0;-0.7];
% est_clock=[8,376.826855];

% 4th
% FilePath.Route='../Data/20210302/4th';
% FilePath.GNSSFile= 'test203024.csv';
% FilePath.INSFile= 'Estelleimu03024.csv';
% FilePath.uboxFile='poslv03024.csv';
%  old_time = 190610;
%  old_est_r_ea_e = [-3961764.933;	3349009.266;	3698312.809];
%  old_est_v_ea_e=[0.000	;0.00	;0.00];
%  attitude_ini = [0;0;(-44)/180*pi];
%  L_ba_b=[0;0;-0.7];
% est_clock=[8,376.826855];

%5th
% FilePath.Route='../Data/20210302/5th';
% FilePath.GNSSFile= 'test2v3.csv';
% FilePath.INSFile= 'Estelleimu03025.csv';
% FilePath.uboxFile='poslv03025.csv';
%  old_time = 194016;
%  old_est_r_ea_e = [-3961764.578;	3349009.716;	3698312.756];
%  old_est_v_ea_e=[0.000	;0.00	;0.00];
%  attitude_ini = [0;0;(-49)/180*pi];
%  L_ba_b=[0;0;-0.7];
% est_clock=[8,376.826855];
 %% Provide initial position, velocity and attitude
old_est_C_b_n=Euler_to_CTM(attitude_ini)';%Coordinate transformation matrix from IMU  to local horizontal coordinate system
[old_est_L_a,old_est_lambda_a,old_est_h_a,old_est_v_ea_n] =...
    pv_ECEF_to_NED(old_est_r_ea_e,old_est_v_ea_e);
[~,~,old_est_C_b_e] = NED_to_ECEF(old_est_L_a,...
    old_est_lambda_a,old_est_h_a,old_est_v_ea_n,old_est_C_b_n);
old_est_r_eb_e=old_est_r_ea_e-old_est_C_b_e*L_ba_b;
old_est_v_eb_e=old_est_v_ea_e;%The lever arm effect of velocity is ignored here
Total_GNSS_epoch = 9435 - 483;
%FilePath.GNSSFile= 'GNSSTCData_noclk_nos.mat';

% Tightly coupled ECEF Inertial navigation and GNSS integrated navigation
[out_profile,out_IMU_bias_est,out_clock,out_KF_SD,PickSubsetRes,RecClockBiasRes,...
    out_MeasurementNoise_SD,out_Resi,InnovationRes,StdInnovationRes] =...
    Tightly_coupled_INS_GNSS(FilePath,old_time,old_est_r_eb_e,old_est_v_eb_e,...
    est_clock,attitude_ini,GNSS_config,TC_KF_config,L_ba_b,Total_GNSS_epoch);

% plotCoor(out_profile(:,2:4));
ubox = csvread(FilePath.uboxFile);
ENUPos2 = ECEF2ENUPos(ubox);
Compar2file([out_profile(2:end,1),out_profile(2:end,11:13)],ENUPos2(1:end,:));