clc,clear;
% %% Constants initial
% %global gnssins;
% deg_to_rad = pi/180;
% rad_to_deg = 1/deg_to_rad;
% c = 299792458;
% ATTITUDE_UPDATE_EULER = 0;		   %/* IMU Attitude update via Euler angles */
% ATTITUDE_UPDATE_QUATERNION = 1;	   %/* IMU Attitude update via quaternions */;
% 
% SET_IMU_LEVELING = 0;				%/* Set initial Roll and Pitch to zero */
% CALC_IMU_LEVELING = 1	;			%/* Calculate Roll and Pitch from accelerometers */
% ALMODE_INS_STATIC = 0	;			%/* IMU Alignment mode: static (average over several epochs) */
% ALMODE_INS_KINEMA = 1	;			%/* IMU Alignment mode: kinematic (only leveling done, heading acquired by GNSS)*/
% SOLQ_FIX = 1;
% 
% GNSS_config.ISBBDS_GPS = 69.13*c/1e9;%2018165 ClockBDS-ClockGPS
% % Satellite to Omit
% GNSS_config.omit = 40:200;%Control the GNSS type involved in the solution through this configuration
% % GNSS type and PRN mapping list
% GNSS_config.GPSPRNList=1:40;
% GNSS_config.GALLIO = 41:80;
% GNSS_config.BDSPRNList=81:130;
% GNSS_config.GLONASSPRNList=131:176;
% 
% gnssins.imuIsAligned = 0;
% gnssins.attitudeUpdateType = 0;
% gnssins.filterUpdateType = 0;
% %gnssins.imuLevelingType = SET_IMU_LEVELING;
% gnssins.imuLevelingType = CALC_IMU_LEVELING;
% gnssins.alignmentType = ALMODE_INS_KINEMA;
% gnssins.imuMeasRate = 0;
% gnssins.gnss_ins_sol.stat = 1;
% gnssins.accBias = zeros(3, 1);
% gnssins.gyrBias = zeros(3, 1);
% gnssins.posXYZ = zeros(3, 1);
% gnssins.velCov = zeros(3, 1);
% gnssins.LL = zeros(2, 1);
% gnssins.velENU = zeros(3, 1);
% gnssins.velXYZ = zeros(3, 1);
% gnssins.R_N = 0.0;
% gnssins.R_E = 0.0;
% gnssins.leverArm = zeros(3, 1);
% gnssins.lastFilterUpdateTime = 0.0;
% gnssins.lastImuTime = 0.0;
% 
% %/* Allocate Strapdown matrices */
% gnssins.previousPos = zeros(3, 1);
% gnssins.previousVel = zeros(3, 1);
% gnssins.attQuat = zeros(4, 1);
% gnssins.attEuler = zeros(3, 1);
% gnssins.w_ie_n = zeros(3, 1);
% gnssins.C_b_n = zeros(3, 3);
% gnssins.C_b_n_prev = zeros(3, 3);
% gnssins.w_en_n = zeros(3, 1);
% gnssins.alpha_ib_b = zeros(3, 1);
% gnssins.w_ib_b = zeros(3, 1);
% gnssins.gVector = zeros(3, 1);
% gnssins.specForce = zeros(3, 1);
% gnssins.velIncrement = zeros(3, 1);
% gnssins.C_b_b = eye(3);
% gnssins.q_b_b = zeros(4, 1);
% gnssins.f_ib_b = zeros(3, 1);
% gnssins.posVel = zeros(6, 1);
% 
% %/* Allocate filter matrices */
% gnssins.xaPriori = zeros(17, 1);
% gnssins.xaPosteriori = zeros(17, 1);
% gnssins.F = zeros(17, 17);
% gnssins.Phi = eye(17);
% gnssins.PaPriori = eye(17);
% gnssins.H_r3_n = zeros(3, 3);
% gnssins.H_v3_n = zeros(3, 3);
% gnssins.H_v5_n = zeros(3, 3);
% 
% gnssins.PaPriori(17,17) = 0.1;
% gnssins.PaPriori(16,16) = 0.1;
% gnssins.PaPriori(15,15) = 0.1;
% gnssins.PaPriori(14,14) = 0.1;
% gnssins.PaPriori(13,13) = 0.1;
% gnssins.PaPriori(12,12) = 0.1;
% gnssins.PaPriori(11,11) = 0.1;
% gnssins.PaPriori(10,10) = 0.1;
% gnssins.PaPriori(9,9) = 0.1;
% gnssins.PaPriori(8,8) = 0.1;
% gnssins.PaPriori(7,7) = 0.1;
% gnssins.PaPriori(6,6) = 10.0;
% gnssins.PaPriori(5,5) = 10.0;
% gnssins.PaPriori(4,4) = 10.0;
% gnssins.PaPriori(3,3) = 10.0;
% gnssins.PaPriori(2,2) = 10.0;
% gnssins.PaPriori(1,1) = 10.0;
% gnssins.PaPosteriori = zeros(17, 17);
% gnssins.Q = eye(17,17);
% gnssins.Q = gnssins.Q .* 0.003;
% gnssins.Q(7,7) = 0;
% gnssins.Q(8,8) = 0;
% gnssins.Q(9,9) = 0;
% 
% % Allocate temporary Matrices
% gnssins.curMat17by17_1 = zeros(17, 17);
% gnssins.curMat17by17_2 = zeros(17, 17);
% gnssins.curMat6by17_1 = zeros(6, 17);
% gnssins.curMat17by6_1 = zeros(17, 6);
% gnssins.curMat6by6_1 = zeros(6, 6);
% gnssins.curMat3by3_1 = zeros(3, 3);
% gnssins.curMat3by3_2 = zeros(3, 3);
% gnssins.curMat3by3_3 = zeros(3, 3);
% gnssins.curMat4by4_1 = zeros(4, 4);
% gnssins.curMat4by4_2 = zeros(4, 4);
% gnssins.curVec3by1_1 = zeros(3, 1);
% gnssins.curVec3by1_2 = zeros(3, 1);
% gnssins.curVec4by1_1 = zeros(4, 1);
% gnssins.curVec4by1_2 = zeros(4, 1);
% gnssins.curVec6by1_1 = zeros(6, 1);
% gnssins.curVec17by1_1 = zeros(17, 1);
% gnssins.curMat17by3_1 = zeros(17, 3);
% gnssins.curMat3by17_1 = zeros(3, 17);
% %% Input
% n  = 0; m = 0;
% 
% GNSSObsFile = "20201105DGNSS-v5.csv";
% RtkFile = "rtk_ecef_alignment_1105.csv"; %File name
% IMUFile = "imu1105.csv";
% load('GNSSTCData.mat'); %TC data
% GNSSinputposformat = "ecef"; % pos input format pos(BLH deg/deg/m) or ecef(m)
% LCputposformat = "pos"; % pos output format pos or ecef
% gnssins.leverArm = gnssins.leverArm; % Offset between IMU and GNSS antenna (in NED)
% 
% % IMU(ENU): Time1, Acc2~4, Gyro5~7
% if exist(IMUFile,'file') ~= 0
%     imudat = csvread(IMUFile);
%     gnssins.imuMeasRate = 50;%IMU freq
%     gnssins.numacc = 2;%Number of acc columns 2~4
%     gnssins.numgyro = 5;%Number of gyro columns 5~7
% end
% % Rtk: Time1, state2, Pos3~5, Vel6~8
% if exist(RtkFile,'file') ~= 0
%     Posdata = csvread(RtkFile);
%     ECEFdata = zeros(length(Posdata)-1,8);
%     n = gnssins.imuMeasRate;
%     GNSSfreq = 1; % GNSS freq
%     if GNSSinputposformat == "pos"
%         for j = 2:length(Posdata)
%             ECEFdata(j,1) = Posdata(j,1);
%             ECEFdata(j,2) = Posdata(j,2);
%             ECEFdata(j,3:4) = Posdata(j,3:4) .* deg_to_rad;
%             ECEFdata(j,5) = Posdata(j,5);
%             ECEFdata(j,3:5) = pos2ecef(ECEFdata(j,3:5));
%             %ECEFdata(j,6:8) = Posdata(j,6:8);
%             ECEFdata(j,6:8) = (ECEFdata(j,3:5) - ECEFdata(j-1,3:5)) ./(ECEFdata(j,1) - ECEFdata(j-1,1)); %if you don;t have speed
%         end
%         ECEFdata = ECEFdata(3:end,:);
%     else
%         ECEFdata = Posdata(:,1:8);
%     end
% end
% % GNSSObs: epoch1, GPSWeek2, GPSSecond3, SNR4, Corrected Pseudo-range5,Satellite pos6~8, range reat 9, rate clock10, , Carricer offset11
% %Sat vel12~14, Elevation angle (deg)15, signal strength of each frequency(dbHz)16~17
% if exist(GNSSObsFile,'file') ~= 0
%     GNSSObs = csvread(GNSSObsFile);
% end
% GNSS_config.mask_SignalStrenth = 30;
% GNSS_config.epoch_interval = 1.0;
% TC_KF_config.RecClockPreprocOptions = 0;
% est_clock=[-2.51158315e-8*c, 8.010831039e-11*c];
% %% Alignment
% GNSSepoch = 1;
% while(gnssins.imuIsAligned ~= 2)
%     
%     m = 0;
%     indexOfFirstImuRec = 0;
%     
%     rtk.sol.time = ECEFdata(GNSSepoch,1);
%     rtk.sol.stat = ECEFdata(GNSSepoch,2);
%     rtk.sol.rr = ECEFdata(GNSSepoch,3:8);
%     
%     for IMUepoch = 1:length(imudat)
%         if (abs(ECEFdata(GNSSepoch,1) - imudat(IMUepoch,1)) <= 0.1)
%             if (m == 0)
%                 imudatt = IMUepoch;
%             end
%             m = m + 1;
%         elseif (imudat(IMUepoch,1) - rtk.sol.time > 0.1)
%             break;
%         end
%     end
%     
%     GNSSepoch = GNSSepoch + 1;
%     % 		if (gnssins.alignmentType == ALMODE_INS_STATIC)
%     % 			%// Not yet implemented
%     % 		end;
%     
%     if (gnssins.alignmentType == ALMODE_INS_KINEMA)
%         if ((n > 0) && (m > 0))
%             if (rtk.sol.stat == SOLQ_FIX)
%                 %/* Convert XYZ to LLH */
%                 for i = 1:3
%                     gnssins.posXYZ(i) = rtk.sol.rr(i);
%                     gnssins.velXYZ(i) = rtk.sol.rr(i + 3);
%                     %gnssins.velCov(i) = rtk.sol.qv(i); 暂无velcov可用
%                 end
%                 gnssins.previousPos = ecef2pos(gnssins.posXYZ);
%                 gnssins.LL(1) = gnssins.previousPos(1);
%                 gnssins.LL(2) = gnssins.previousPos(2);
%                 gnssins.velENU = ecef2enu(gnssins.LL, gnssins.velXYZ);
%                 totalVel = sqrt(power(gnssins.velENU(1), 2.0) + power(gnssins.velENU(2), 2.0) + power(gnssins.velENU(3), 2.0));
%                 %totalVelAccuracy = sqrt(gnssins.velCov(1) + gnssins.velCov(2) + gnssins.velCov(3));%有速度协方差时使用
%                 %/* Determine if vehicle is stationary or moving. These thresholds are an empiric assumption. */
%                 %if ((totalVel > 4.0 * totalVelAccuracy) && (totalVelAccuracy < 0.3) && (totalVel > 3.0))%有速度协方差时使用
%                 if ((totalVel > 3.0))
%                     indexOfFirstImuRec = findImuMeas(rtk, imudat,m,gnssins, imudatt);
%                     %/* Alignment Phase */
%                     %/* Levelling */
%                     gnssins.gVector =normalGravity(gnssins.previousPos(1), gnssins.previousPos(3));
%                     gamma = gnssins.gVector(3);
%                     if (gnssins.imuLevelingType == CALC_IMU_LEVELING)
%                         gnssins.attEuler(2) = asin((imudat(indexOfFirstImuRec ,gnssins.numacc) - gnssins.accBias(1)) / gamma);
%                         gnssins.attEuler(1) = atan2(imudat(indexOfFirstImuRec ,gnssins.numacc + 1) - gnssins.accBias(2), imudat(indexOfFirstImuRec ,gnssins.numacc + 2) - gnssins.accBias(3));
%                     elseif (gnssins.imuLevelingType == SET_IMU_LEVELING)
%                         gnssins.attEuler(1) = 0.0;
%                         gnssins.attEuler(2) = 0.0;
%                     end
%                     %/* Calculate Initial Heading from GNSS velocities (procedure for MEMS IMUs), according to (2) */
%                     gnssins.attEuler(3) = atan2(gnssins.velENU(1), gnssins.velENU(2));
%                     %/* Set initial position and velocity */
%                     gnssins.previousVel(1) = gnssins.velENU(2);
%                     gnssins.previousVel(2) = gnssins.velENU(1);
%                     gnssins.previousVel(3) = -gnssins.velENU(3);
%                     gnssins.imuIsAligned = 2;
%                     %/* Initialize C_b_n from Euler angles */
%                     gnssins.C_b_n = body2navFrame(gnssins.attEuler(1), gnssins.attEuler(2), gnssins.attEuler(3));
%                     %/* Initialize quaternion from euler angles */
%                     gnssins.attQuat = eul2quat(gnssins.attEuler);
%                     %/* Initial position and attitude is determined, filter is initialized, propagate filter and states to next epoch */
%                     for i = (indexOfFirstImuRec:m)
%                         gnssins = doStrapdown(imudat(i,:), gnssins, 1.0/gnssins.imuMeasRate);
%                         gnssins = propagateFilter(gnssins, 1.0 / gnssins.imuMeasRate);
%                     end
%                 end
%             end
%         end
%     end
% end
%% Main loop
        %% init
deg_to_rad = pi/180;
rad_to_deg = 1/deg_to_rad;
c = 299792458;
micro_g_to_meters_per_second_squared = 9.80665E-6;
% Interval between GNSS epochs (s)
GNSS_config.epoch_interval = 0.2;
% Max Number of satellites
GNSS_config.intend_no_GNSS_meas = 30;%If the number of observations exceeds this value, start satellite screening
% Mask angle (deg)
GNSS_config.mask_angle = 7;
% Mask Carrier to noise ratio(dbhz)
GNSS_config.mask_SignalStrenth = 30;
% Inner System Bias(m)
% GNSS_config.ISBBDS_GPS = 59.5*c/1e9;%2018206 ClockBDS-ClockGPS
GNSS_config.ISBBDS_GPS = 69.13*c/1e9;%2018165 ClockBDS-ClockGPS
% Satellite to Omit
GNSS_config.omit = 33:56;%Control the GNSS type involved in the solution through this configuration
% GNSS type and PRN mapping list
GNSS_config.GPSPRNList=1:32;
GNSS_config.GLONASSPRNList=33:56;
GNSS_config.BDSPRNList=57:91;

% Initial attitude uncertainty per axis (deg, converted to rad)
TC_KF_config.init_att_unc = degtorad(20);
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
TC_KF_config.gyro_noise_PSD = (0.01)^2;
% Accelerometer noise PSD (micro-g^2 per Hz, converted to m^2 s^-3)                
TC_KF_config.accel_noise_PSD = (0.1)^2;

% NOTE: A large noise PSD is modeled to account for the scale-factor and
% cross-coupling errors that are not directly included in the Kalman filter model
% Accelerometer bias random walk PSD (m^2 s^-5)
TC_KF_config.accel_bias_PSD = 1.0E-5;
% Gyro bias random walk PSD (rad^2 s^-3)
TC_KF_config.gyro_bias_PSD = 4.0E-11;
% Receiver clock frequency-drift PSD (m^2/s^3)
TC_KF_config.clock_freq_PSD = 1;
% Receiver clock phase-drift PSD (m^2/s)
TC_KF_config.clock_phase_PSD = 1;
% Pseudo-range measurement noise SD (m)
TC_KF_config.pseudo_range_SD = 2.5;
% Pseudo-range rate measurement noise SD (m/s)
TC_KF_config.range_rate_SD = 0.1;
TC_KF_config.RecClockPreprocOptions=0;% The default value is 0,If the receiver clock jitter, the value is 2
TC_KF_config.KFMethod='M-LSKF';%ClassicKF means using the classic Kalman filter,M-LSKFClassicKF means using the classic Kalman filter,，IAE-KF表示使用新息对R阵进行膨胀
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
old_time=364516;%20201105 alignment time
old_est_r_ea_e=[-3961765.2156 ;  3349009.3056;  3698311.6652];%antenna position
old_est_v_ea_e=[0;0;0];
%old_est_v_ea_e=[0.002236;0.034801;0.002225];%receiver velocity
est_clock=[-0.007080535752941*c, 3.487476978027949e+02];
%init attitude roll, pitch, yaw convert to yaw、pitch、roll
attitude_ini = [-0.0025;-0.0016;-0.7696];
L_ba_b=[0;0;0];
old_est_C_b_n=Euler_to_CTM(attitude_ini)';%Coordinate transformation matrix from IMU  to local horizontal coordinate system
[old_est_L_a,old_est_lambda_a,old_est_h_a,old_est_v_ea_n] =...
    pv_ECEF_to_NED(old_est_r_ea_e,old_est_v_ea_e);
[~,~,old_est_C_b_e] = NED_to_ECEF(old_est_L_a,...
    old_est_lambda_a,old_est_h_a,old_est_v_ea_n,old_est_C_b_n);
old_est_r_eb_e=old_est_r_ea_e-old_est_C_b_e*L_ba_b;
old_est_v_eb_e=old_est_v_ea_e;%The lever arm effect of velocity is ignored here
Total_GNSS_epoch = 9435 - 483;
%FilePath.GNSSFile= 'GNSSTCData_noclk_nos.mat';
FilePath.GNSSFile= 'GNSSTCData_GQ.mat';
FilePath.INSFile= 'IMUData.mat';
% Tightly coupled ECEF Inertial navigation and GNSS integrated navigation
[out_profile,out_IMU_bias_est,out_clock,out_KF_SD,PickSubsetRes,RecClockBiasRes,...
    out_MeasurementNoise_SD,out_Resi,InnovationRes,StdInnovationRes] =...
    Tightly_coupled_INS_GNSS(FilePath,old_time,old_est_r_eb_e,old_est_v_eb_e,...
    est_clock,attitude_ini,GNSS_config,TC_KF_config,L_ba_b,Total_GNSS_epoch);