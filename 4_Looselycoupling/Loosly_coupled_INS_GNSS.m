function [out_profile,out_IMU_bias_est,out_KF_SD,out_MeasurementNoise_SD,out_Resi,InnovationRes,StdInnovationRes] =...
    Loosly_coupled_INS_GNSS(FilePath,old_time,old_est_r_eb_e,old_est_v_eb_e,attitude_ini,GNSS_config,TC_KF_config,L_ba_b,Total_GNSS_epoch)
%Tightly_coupled_INS_GNSS - Simulates inertial navigation using ECEF
% navigation equations and kinematic model, GNSS and tightly coupled
% INS/GNSS integration.
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 12/4/2012 by Paul Groves
% Inputs:
%   FilePath.GNSSFile   GNSS Obs File For Tightly_coupled_INS_GNSS
%       % Format of GNSS observation array:
%           Column 1: epoch
%           Column 2: Obsweek (GPS week)
%           Column 3: Obssec (GPS second)
%           Column 4: PRN (PRN=GPS_PRN=GLONASS_PRN+32=BDS_PRN+56)
%           Column 5: Ionospheric Free pseudorange linear combination (m)
%           Column 6: slant tropospheric delay (m)
%           Column 7: Satellite clock error (code bias have been correted) (s)
%           Column 8: relativity correction (m)
%           Column 9-11: Satellite position in ECEF(Earth rotation correction have been made,ECEF at Receiving moment)(m)
%           Column 12: range rate(m/s)
%           Column 13: Rate of Satellite clock  (s/s)
%           Column 14-16: Satellite velocity in ECEF(Earth rotation correction have been made,ECEF at Receiving moment)(m/s)
%           Column 17: flag= 0 means this PRN was Removed in  GNSS Single point Least squares Solution
%           Column 18: Elevation angle (deg)
%           Column 19: Azimuth (deg)
%           Column 20: user ranging error (By RTK solution) (m)
%           Column 21: GNSS Single point Least squares Solution Residual (m)
%           Column 22: a priori Pseudo-distance noise standard deviation (m)
%           Column 23-24:signal strength of each frequency(dbHz)
%           Column 25£ºIonosphere delay by KLOBUCHAR(if Available)(m)
%   FilePath.INSFile    INS Obs File For Tightly_coupled_INS_GNSS
%       % Format of INS observation array:
%           Column 1: Obssec (GPS second)
%           Column 2: gyroscope observation (Forward) (rad/s)
%           Column 3: gyroscope observation (Right) (rad/s)
%           Column 4: gyroscope observation (Down) (rad/s)
%           Column 5: accelerometer observation (Forward) (m*s-2)
%           Column 6: accelerometer observation (Right) (m*s-2)
%           Column 7: accelerometer observation (Down) (m*s-2)
%   old_time,old_est_r_eb_e,old_est_v_eb_e,est_clock,attitude_ini
%       Initial alignment result
%   GNSS_config
%       defined in INS_GNSS_Demo_7
%   TC_KF_config
%       defined in INS_GNSS_Demo_7
%   L_ba_b
%       The vector from the IMU center to the phase center of the GNSS antenna (forward, right, down)
%   Total_GNSS_epoch
%       Number of GNSS observations
% Outputs:
%   out_profile        Navigation solution as a motion profile array
%        Format of output IMU biases array:
%          Column 1: time (sec)
%          Column 2: estimated ECEF-X position (m)
%          Column 3: estimated ECEF-Y position (m)
%          Column 4: estimated ECEF-Z position (m)
%          Column 5: estimated ECEF-X velocity (m/s)
%          Column 6: estimated ECEF-Y velocity (m/s)
%          Column 7: estimated ECEF-Z velocity (m/s)
%          Column 8: estimated roll angle (rad)
%          Column 9: estimated pitch angle (rad)
%          Column 10: estimated yaw angle (rad)
%          Column 11: estimated Local geodetic frame (origin defined by STA ) North position (m)
%          Column 12: estimated Local geodetic frame (origin defined by STA ) East position (m)
%          Column 13: estimated Local geodetic frame (origin defined by STA ) Up position (m)
%   out_IMU_bias_est   Kalman filter IMU bias estimate array
% Format of output IMU biases array:
%  Column 1: time (sec)
%  Column 2: estimated X accelerometer bias (m/s^2)
%  Column 3: estimated Y accelerometer bias (m/s^2)
%  Column 4: estimated Z accelerometer bias (m/s^2)
%  Column 5: estimated X gyro bias (rad/s)
%  Column 6: estimated Y gyro bias (rad/s)
%  Column 7: estimated Z gyro bias (rad/s)
%   out_clock          GNSS Receiver clock estimate array
% Format of receiver clock array:
%  Column 1: time (sec)
%  Column 2: estimated clock offset (m)
%  Column 3: estimated clock drift (m/s)
%   out_KF_SD          Output Kalman filter state uncertainties
% Format of KF state uncertainties array:
%  Column 1: time (sec)
%  Column 2: X attitude error uncertainty (rad)
%  Column 3: Y attitude error uncertainty (rad)
%  Column 4: Z attitude error uncertainty (rad)
%  Column 5: X velocity error uncertainty (m/s)
%  Column 6: Y velocity error uncertainty (m/s)
%  Column 7: Z velocity error uncertainty (m/s)
%  Column 8: X position error uncertainty (m)
%  Column 9: Y position error uncertainty (m)
%  Column 10: Z position error uncertainty (m)
%  Column 11: X accelerometer bias uncertainty (m/s^2)
%  Column 12: Y accelerometer bias uncertainty (m/s^2)
%  Column 13: Z accelerometer bias uncertainty (m/s^2)
%  Column 14: X gyro bias uncertainty (rad/s)
%  Column 15: Y gyro bias uncertainty (rad/s)
%  Column 16: Z gyro bias uncertainty (rad/s)
%  Column 17: clock offset uncertainty (m)
%  Column 18: clock drift uncertainty (m/s)
%   PickSubsetRes      PDOP ¡¢Num of Sat¡¢time cost of Satellite Selection
%   RecClockBiasRes    Store the difference between the inverse-calculated receiver clock difference of each satellite and RecClockPre
%   InnovationRes      innovation Residual£¬ RecClockPreprocOptions==2 means same as RecClockBiasRes
%   out_MeasurementNoise_SD     measurement noise SD output record(m)
%   out_Resi            Residual record
%   StdInnovationRes  Standardized innovation
% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details
% Modified 2017/8 by LiuXiao BUAA benzenemo@buaa.edu.cn % 20170311B104ZXY
% Begins

STA.STA(1).Coor(1:3) = [-3961904.939,3348993.763,3698211.764];
GNSSObs = csvread(FilePath.GNSSFile);
IMUData = csvread(FilePath.INSFile);
GNSSObsVel = csvread(FilePath.GNSSVelFile);
%% Estelle imu body frame R-F-D to F-R-D
IMUData_ = IMUData;
IMUData_(:,2:4) = IMUData_(:,2:4) ;%.* 9.7978
IMUData_(:,5:7) = IMUData_(:,5:7)./180*pi;
IMUData_(:,2:7) = IMUData_(:,2:7) - mean(IMUData_(500:3000,2:7)); %remove bias
IMUData(:,1) = IMUData_(:,1) ;%+18
IMUData(:,3) = IMUData_(:,2);
IMUData(:,2) = IMUData_(:,3);
IMUData(:,4) = -IMUData_(:,4) -9.7978;
IMUData(:,6) = IMUData_(:,5);
IMUData(:,5) = IMUData_(:,6);
IMUData(:,7) = -IMUData_(:,7) ;
IMUData(:,9) =  IMUData_(:,9) .* 1000/3600/1.41;
% CarSpeed = [IMUData_(:,1)+7.2,IMUData_(:,9).*1000/3600,IMUData_(:,11)];
% IMUData(:,8) = CarSpeed(:,2);

%% G370 imu body frame B-L-D to F-R-D
% IMUData_ = IMUData;
% IMUData_(:,5:7) = IMUData_(:,5:7)./180*pi;
% IMUData_(:,2:7) = IMUData_(:,2:7) - mean(IMUData_(1500:4000,2:7)); %remove bias
% IMUData(:,1) =  IMUData_(:,1);
% IMUData(:,2) = -IMUData_(:,2);
% IMUData(:,3) = -IMUData_(:,3);
% IMUData(:,4) = -IMUData_(:,4) - 9.7978;
% IMUData(:,5) = -IMUData_(:,5);
% IMUData(:,6) = -IMUData_(:,6);
% IMUData(:,7) = -IMUData_(:,7);

tor_s = 0.02; %IMU freq
IMU = IMUData(find(IMUData(:,1) == old_time):end,:);

[no_epochs,~]=size(IMU);
no_epochs = find(IMU(:,1) == min(GNSSObs(end,1),IMU(end,1)));
no_epochs = round(no_epochs/1.75);
%% Check Euler angle
% figure (1);
% plotEuler(IMU(1:end,:));
c = 299792458;
GM = 3.986005000000000e+14;

% Determine Least-squares GNSS position solution
[old_est_L_b,old_est_lambda_b,old_est_h_b,old_est_v_eb_n] =...
    pv_ECEF_to_NED(old_est_r_eb_e,old_est_v_eb_e);
est_L_b = old_est_L_b;
est_lambda_b=old_est_lambda_b;
% Initialize estimated attitude solution
old_est_C_b_n=Euler_to_CTM(attitude_ini)';% To transformation matrix
[~,~,old_est_C_b_e] = NED_to_ECEF(old_est_L_b,...
    old_est_lambda_b,old_est_h_b,old_est_v_eb_n,old_est_C_b_n);

% Initialize output profile record and errors record
out_profile = zeros(no_epochs,13);
out_IMU_bias_est=zeros(Total_GNSS_epoch,7);
% out_clock=zeros(Total_GNSS_epoch,4);
out_KF_SD=zeros(Total_GNSS_epoch,18);
% PickSubsetRes=zeros(Total_GNSS_epoch,4);
[GNSSObsNum,~]=size(GNSSObs);

InnovationRes=zeros(GNSSObsNum,5);
StdInnovationRes=zeros(GNSSObsNum,5);
out_MeasurementNoise_SD=zeros(GNSSObsNum,4);
out_Resi=zeros(GNSSObsNum,7);
% Generate output profile record
out_profile(1,1) = old_time + 0.2;
out_profile(1,2:4)=(old_est_r_eb_e+old_est_C_b_e*L_ba_b)';
out_profile(1,5:7) = old_est_v_eb_e';
out_profile(1,8:10) = CTM_to_Euler(old_est_C_b_n')';
Rotation=[   -sin(est_L_b)*cos(est_lambda_b),-sin(est_L_b)*sin(est_lambda_b),cos(est_L_b);
    -sin(est_lambda_b),cos(est_lambda_b),0;
    cos(est_L_b)*cos(est_lambda_b),cos(est_L_b)*sin(est_lambda_b),sin(est_L_b)];
NEU=Rotation*(out_profile(1,2:4)-STA.STA(1).Coor(1:3))';
out_profile(1,11:13)=NEU';

% Determine errors and generate output record
% Initialize Kalman filter P matrix and IMU bias states
P_matrix = Initialize_TC_P_matrix(TC_KF_config);
est_IMU_bias = zeros(6,1);
est_ZUPT_IMU_bias = zeros(6,1);
% Generate IMU bias and clock output records
out_IMU_bias_est(1,1) = old_time;
out_IMU_bias_est(1,2:7) = est_IMU_bias';
% Generate KF uncertainty record
out_KF_SD(1,1) = old_time;
for i =1:15
    out_KF_SD(1,i+1) = sqrt(P_matrix(i,i));
end % for i
% Initialize GNSS model timing
time_last_GNSS = round(old_time,1);%GNSSObs update time
TC_KF_config.StationarityFlag = 0;%station flag£¬defult is 0, kinmetic
GNSS_epoch = 1;
% Progress bar
dots = '....................';
bars = '||||||||||||||||||||';
rewind = '\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b';
fprintf(strcat('Processing: ',dots));
progress_mark = 0;
progress_epoch = 0;
debug = [];
oldFixPos=[0,0,0];
GNSS_index_fix = [];
%% Main loop
for epoch = 1:no_epochs
    % Update progress bar
    if (epoch - progress_epoch) > (no_epochs/20)
        progress_mark = progress_mark + 1;
        progress_epoch = epoch;
        fprintf(strcat(rewind,bars(1:progress_mark),...
            dots(1:(20 - progress_mark))));
    end % if epoch
    
    % Input data from motion profile
    time=IMU(epoch,1);
    
    % Time interval
    tor_i = time - old_time;
    
    % Calculate specific force and angular rate
    meas_f_ib_b_ =IMU(epoch,2:4)';
    meas_omega_ib_b_ =IMU(epoch,5:7)';
    CarSpeed_ = IMU(epoch,9);
    
    % Correct IMU errors
    meas_f_ib_b = meas_f_ib_b_ - est_IMU_bias(1:3);
    meas_omega_ib_b = meas_omega_ib_b_ - est_IMU_bias(4:6);
    
    % Update estimated navigation solution
    [est_r_eb_e,est_v_eb_e,est_C_b_e] = Nav_equations_ECEF(tor_i,...
        old_est_r_eb_e,old_est_v_eb_e,old_est_C_b_e,meas_f_ib_b,...
        meas_omega_ib_b);
    lcflag = 0;
    index_GNSS=find(round(GNSSObs(:,1),1)==round(time,1));
    index_GNSSVel=find(round(GNSSObsVel(:,1),1)==round(time,1));
    
    % Determine whether to update GNSS observation and run Kalman filter
    if ((time - time_last_GNSS) >= GNSS_config.epoch_interval &&  ~isempty(index_GNSS) && ~isempty(index_GNSSVel))%&& ~isempty(index_GNSSVel)
        %According to the observation time
        %         index_GNSS=find(GNSSObs(:,1)==floor(time));
        %             GNSSObs(:,3) = round(GNSSObs(:,3),2);%Take two decimal places
        GNSS_epoch = GNSS_epoch + 1;
        tor_s = time - time_last_GNSS;  % KF time interval
        time_last_GNSS = round(time,1);
        FIXFlag = GNSSObs(index_GNSS,2);
        GNSS_r_eb_e = GNSSObs(index_GNSS,3:5)';
        %         GNSS_v_eb_e =  GNSSObs(index_GNSS,6:8)';
        %         GNSS_v_eb_e = GNSS_v_eb_e*CarSpeed_/norm(GNSS_v_eb_e);
        GNSS_v_eb_e =  GNSSObsVel(index_GNSSVel,2:4)';
        GNSS_Cov = max(GNSSObs(index_GNSS,7:9));
        [~,~,~,v_eb_n,est_C_b_n] = ECEF_to_NED(GNSS_r_eb_e,GNSS_v_eb_e,old_est_C_b_e);
        GNSSYaw = atan2(v_eb_n(2),v_eb_n(1));
        
        if strcmp(TC_KF_config.KFMethod,'ClassicKF')
            % Run Integration Kalman filter
            debug = [debug;time_last_GNSS,GNSS_r_eb_e',GNSS_v_eb_e',est_r_eb_e',est_v_eb_e',CTM_to_Euler(est_C_b_n')'...
                .* 180/pi,est_IMU_bias',meas_f_ib_b_',norm(GNSS_v_eb_e),CarSpeed_,GNSSYaw*180/pi];%Debug ,CalHeading(old_est_r_eb_e,GNSS_r_eb_e)
            if epoch>= 500 && FIXFlag == 1
                %                 GNSS_index_old = find(round(GNSSObs(:,1),1)==round(time,1));
                
                GNSS_index_fix = [GNSS_index_fix;index_GNSS];
                if length(GNSS_index_fix) >10
                    if ~isnan(index_GNSS)
                        az_ = abs(CalHeading(GNSSObs(GNSS_index_fix(end-1),(3:5)),GNSSObs(GNSS_index_fix(end),(3:5)))-CalHeading(GNSSObs(GNSS_index_fix(end-5),(3:5)),GNSSObs(GNSS_index_fix(end-1),(3:5))));
                        if az_ > 300
                            az_ = 360 - az_ ;
                        end
                        if az_ >28 && CarSpeed_ >4
%                             index_GNSS
                            GNSS_index_fix=GNSS_index_fix(1:end-1,:);
                            %                         GNSSObs(GNSS_index_fix(end),2) = 2;
%                             FIXFlag = 4;
                        end
                    end
                    
                end
            end
            % Adaptative KF
            if (abs(GNSS_Cov) >= 2.2) && FIXFlag == 2
                FIXFlag = 3;
            elseif (abs(norm(GNSS_v_eb_e)) < 3 || CarSpeed_ < 3) && FIXFlag == 2
                %                 FIXFlag = 4;
                %             elseif epoch>= 500 && az_ > 30 && FIXFlag == 1 && CarSpeed_ > 4
                % %                 time_last_GNSS
                %                 FIXFlag = 5;
            elseif (abs(GNSS_Cov) < 0.5) && FIXFlag == 2
                FIXFlag = 1;
            end
            
            [est_C_b_e,est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix] =...
                LC_KF_Epoch(GNSS_r_eb_e,GNSS_v_eb_e,tor_s,est_C_b_e,...
                est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix,meas_f_ib_b,est_L_b,TC_KF_config,FIXFlag);
        end
        %% Generate IMU bias and clock output records
        out_IMU_bias_est(GNSS_epoch,1) = time;
        out_IMU_bias_est(GNSS_epoch,2:7) = est_IMU_bias';
        % Generate KF uncertainty output record
        out_KF_SD(GNSS_epoch,1) = time;
        for i =1:15
            out_KF_SD(GNSS_epoch,i+1) = sqrt(P_matrix(i,i));
        end % for i
        %measurement noise SD output record(m)
        if ~strcmp(TC_KF_config.KFMethod,'ClassicKF')
            out_MeasurementNoise_SD(index_GNSS,:)=[GNSSObs(index_GNSS,2:4),sqrt(diag(R_matrix(1:no_GNSS_meas,1:no_GNSS_meas)))];
        end
        lcflag = 1;
    elseif ((time - time_last_GNSS) >= GNSS_config.epoch_interval &&  isempty(index_GNSS) && ~isempty(index_GNSSVel))
        GNSS_epoch = GNSS_epoch + 1;
        tor_s = time - time_last_GNSS;  % KF time interval
        time_last_GNSS = round(time,1);
        GNSS_v_eb_e =  GNSSObsVel(index_GNSSVel,2:4)';
        %         GNSS_v_eb_e = GNSS_v_eb_e*CarSpeed_/norm(GNSS_v_eb_e);
        debug = [debug;time_last_GNSS,est_r_eb_e',GNSS_v_eb_e',est_r_eb_e',est_v_eb_e',CTM_to_Euler(est_C_b_n')'...
            .* 180/pi,est_IMU_bias',meas_f_ib_b_',norm(GNSS_v_eb_e),CarSpeed_,GNSSYaw*180/pi];%Debug CalHeading(old_est_r_eb_e,GNSS_r_eb_e)
        [est_C_b_e,est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix] =...
            LC_VEL_KF_Epoch(GNSS_v_eb_e,tor_s,est_C_b_e,...
            est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix,meas_f_ib_b,est_L_b,TC_KF_config,TC_KF_config.vel_meas_SD^2 );
        lcflag = 2;
        %             elseif (((time - time_last_GNSS) >= GNSS_config.epoch_interval) &&  isempty(index_GNSS) &&  ~isempty(CarSpeed_)) %&& CarSpeed_ == 0
        % %         %         if norm(GNSS_v_eb_e) < 1
        % %                     [L_b,lambda_b,h_b,v_eb_n,est_C_b_n] = ECEF_to_NED(est_r_eb_e,est_v_eb_e,est_C_b_e);
        % %         %             Euler_ = CTM_to_Euler(est_C_b_n');
        % %         %             YawAngle_ = Euler_(3);
        % %         %         else
        % %                     YawAngle_ = GNSSYaw;
        % %         %         end
        % %                 CarSpeedNED = [CarSpeed_ * cos(YawAngle_);CarSpeed_ * sin(YawAngle_);0];
        % %                 [~,CarVel,CarC_b_e] = NED_to_ECEF(L_b,lambda_b,h_b,CarSpeedNED,est_C_b_n);
        % %                 CarVel = GNSS_v_eb_e .* CarSpeed_ /norm(GNSS_v_eb_e);
        %                 [est_C_b_e,est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix] =...
        %                     LC_VEL_KF_Epoch(GNSS_v_eb_e,tor_s,est_C_b_e,...
        %                     est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix,meas_f_ib_b,-est_L_b,TC_KF_config,TC_KF_config.CarSpeed^2);
        %                 lcflag = 3;
    end % if time
    
    %% Convert navigation solution to NED
    [est_L_b,est_lambda_b,~,~,est_C_b_n] =...
        ECEF_to_NED(est_r_eb_e,est_v_eb_e,est_C_b_e);
    
    % Generate output profile record
    out_profile(epoch,1) = time;
    out_profile(epoch,2:4)=(est_r_eb_e+est_C_b_e*L_ba_b)';%Position of antenna
    out_profile(epoch,5:7) = (est_v_eb_e+est_C_b_e*( Skew_symmetric(meas_omega_ib_b)*L_ba_b))';%Antenna velocity
    out_profile(epoch,8:10) = CTM_to_Euler(est_C_b_n')' .* 180/pi;
    out_profile(epoch,11) = lcflag;
    %     Rotation=[   -sin(est_L_b)*cos(est_lambda_b),-sin(est_L_b)*sin(est_lambda_b),cos(est_L_b);
    %         -sin(est_lambda_b),cos(est_lambda_b),0;
    %         cos(est_L_b)*cos(est_lambda_b),cos(est_L_b)*sin(est_lambda_b),sin(est_L_b)];
    %     NEU=Rotation*(out_profile(epoch,2:4)-STA.STA(1).Coor(1:3))';
    %     out_profile(epoch,11:13)=NEU';
    
    % Reset old values
    old_time = time;
    old_est_r_eb_e = est_r_eb_e;
    old_est_v_eb_e = est_v_eb_e;
    old_est_C_b_e = est_C_b_e;
end %epoch

% figure (2);
% plot(out_profile(:,8:10));

% Complete progress bar
fprintf(strcat(rewind,bars,'\n'));

% Ends