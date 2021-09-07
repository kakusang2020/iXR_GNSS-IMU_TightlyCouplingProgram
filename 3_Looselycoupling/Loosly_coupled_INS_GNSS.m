function [out_profile,out_IMU_bias_est,out_KF_SD,out_MeasurementNoise_SD,out_Resi,InnovationRes,StdInnovationRes] =...
    Loosly_coupled_INS_GNSS(FilePath,old_time,old_est_r_eb_e,old_est_v_eb_e,attitude_ini,GNSS_config,LC_KF_config,L_ba_b,Total_GNSS_epoch)
% Loosely_coupled_INS_GNSS - Simulates inertial navigation using ECEF
% navigation equations and kinematic model, GNSS and loosely coupled
% INS/GNSS integration.
%
% Software for use with "Principles of GNSS, Inertial, and Multisensor
% Integrated Navigation Systems," Second Edition.
%
% This function created 12/4/2012 by Paul Groves
% Inputs:
%   FilePath.GNSSFile   GNSS Obs File For loosely_coupled_INS_GNSS
%       % Format of GNSS observation array:
%           Column 1: Obssec (GPS second)
%           Column 2-4: position in ECEF
%           Column 5: Quality (1 Fix, 2 Float)
%           Column 6: Number of satellite
%           Column 7-12: stander deviation in x,y,z xy,yz,zx(m)
%           Column 13: age(s)
%           Column 14: ratio
%   FilePath.GNSSVelFile   GNSS Obs velocity File For Loosely_coupled_INS_GNSS
%       % Format of GNSS observation array:
%           Column 1: Obssec (GPS second)
%           Column 2-4: velocity in ECEF
%   FilePath.INSFile    INS Obs File For loosely_coupled_INS_GNSS
%       % Format of INS observation array:
%           Column 1: Obssec (GPS second)
%           Column 2: accelerometer observation (Forward) (m*s-2)
%           Column 3: accelerometer observation (Right) (m*s-2)
%           Column 4: accelerometer observation (Down) (m*s-2)
%           Column 5: gyroscope observation (Forward) (rad/s)
%           Column 6: gyroscope observation (Right) (rad/s)
%           Column 7: gyroscope observation (Down) (rad/s)
%   old_time,old_est_r_eb_e,old_est_v_eb_e,est_clock,attitude_ini
%       Initial alignment result
%   GNSS_config
%       defined in Main_LC
%   TC_KF_config
%       defined in Main_LC
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
%   out_MeasurementNoise_SD     measurement noise SD output record(m)
%   out_Resi            Residual record
%   StdInnovationRes  Standardized innovation
% Copyright 2012, Paul Groves
% License: BSD; see license.txt for details
% Begins
addpath(FilePath.Route);% add data path
STA.STA(1).Coor(1:3) = [-3961904.939,3348993.763,3698211.764];
GNSSObs = csvread(FilePath.GNSSFile);
IMUData = csvread(FilePath.INSFile);
GNSSObsVel = csvread(FilePath.GNSSVelFile);
% Sensor = csvread(FilePath.SensorFile);
%% Estelle imu body frame R-F-D to F-R-D
if strfind(FilePath.INSFile,'Estelle')
    IMUData_ = IMUData;
    IMUData_(:,2:4) = IMUData_(:,2:4) ;%.* 9.7978
    IMUData_(:,5:7) = IMUData_(:,5:7)./180*pi;
    IMUData_(:,2:7) = IMUData_(:,2:7) - mean(IMUData_(100:4000,2:7)); %remove bias
    IMUData(:,1) = IMUData_(:,1) ;%+18
    IMUData(:,3) = IMUData_(:,2);
    IMUData(:,2) = IMUData_(:,3);
    IMUData(:,4) = -IMUData_(:,4) -9.7978;
    IMUData(:,6) = IMUData_(:,5);
    IMUData(:,5) = IMUData_(:,6);
    IMUData(:,7) = -IMUData_(:,7) ;
    %% G370 imu body frame B-L-D to F-R-D
elseif strfind(FilePath.INSFile,'G370')
    IMUData_ = IMUData;
    IMUData_(:,5:7) = IMUData_(:,5:7)./180*pi;
%     IMUData_(:,2:7) = IMUData_(:,2:7) - mean(IMUData_(4000:10000,2:7)); %remove bias
    IMUData(:,1) =  IMUData_(:,1);
    IMUData(:,2) = -IMUData_(:,2);
    IMUData(:,3) = -IMUData_(:,3);
    IMUData(:,4) = IMUData_(:,4) - 9.7978;
    IMUData(:,5) = IMUData_(:,5);
    IMUData(:,6) = IMUData_(:,6);
    IMUData(:,7) = -IMUData_(:,7);
else
    disp('INS file name not correct!');
    return
end

tor_s = 0.02; %IMU freq
IMU = IMUData(find(IMUData(:,1) == old_time):end,:);

[no_epochs,~]=size(IMU);
no_epochs = find(IMU(:,1) == min(GNSSObs(end,1),IMU(end,1)));
% no_epochs = round(no_epochs/1.75);
%% Check Euler angle (YAW)
if LC_KF_config.debug
    old_est_v_eb_e_ = [];
    imuHeading=0;
    imuvel=zeros(length(IMU),2);
    imupos=zeros(length(IMU),2);
    for i =1:length(GNSSObsVel)
        [~,~,~,old_est_v_eb_n] =pv_ECEF_to_NED(old_est_r_eb_e,GNSSObsVel(i,5:7)');
        old_est_v_eb_e_=[old_est_v_eb_e_;GNSSObsVel(i,1),atan2(old_est_v_eb_n(2),old_est_v_eb_n(1)).*180/pi];
    end
    figure (1);
    plotEuler(IMU(1:end,:),(attitude_ini(3))*180/pi);%-0.5*pi
    hold on
    plot(old_est_v_eb_e_(:,1),old_est_v_eb_e_(:,2));
end

% for i=2:length(IMU)
%     imuHeading=imuHeading+IMU(i,7)*tor_s;
%     imuvel(i,1)= imuvel(i-1,1)+sin(imuHeading)*(IMU(i-1,2)+IMU(i,2))/2*tor_s;
%     imuvel(i,2)= imuvel(i-1,2)+cos(imuHeading)*(IMU(i-1,2)+IMU(i,2))/2*tor_s;
%     imupos(i,1)= imupos(i-1,1)+imuvel(i,1)*tor_s;
%     imupos(i,2)= imupos(i-1,2)+imuvel(i,2)*tor_s;
% end
% figure (2);
% plot(imupos(:,1),imupos(:,2));
% Determine Least-squares GNSS position solution
[old_est_L_b,old_est_lambda_b,old_est_h_b,old_est_v_eb_n] =pv_ECEF_to_NED(old_est_r_eb_e,old_est_v_eb_e);
est_L_b = old_est_L_b;
est_lambda_b=old_est_lambda_b;
% Initialize estimated attitude solution
old_est_C_b_n=Euler_to_CTM(attitude_ini)';% To transformation matrix
[~,~,old_est_C_b_e] = NED_to_ECEF(old_est_L_b,old_est_lambda_b,old_est_h_b,old_est_v_eb_n,old_est_C_b_n);

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
out_profile(1,1) = old_time;
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
P_matrix = Initialize_LC_P_matrix(LC_KF_config);
est_IMU_bias = zeros(6,1);
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
LC_KF_config.StationarityFlag = 0;%station flag£¬defult is 0, kinmetic
GNSS_epoch = 1;
% Progress bar
dots = '....................';
bars = '||||||||||||||||||||';
rewind = '\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b';
fprintf(strcat('Processing: ',dots));
progress_mark = 0;
progress_epoch = 0;
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
    %     Sensor_=Sensor(find(round(Sensor(:,1),2)==round(time,2)),2);
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
        GNSS_epoch = GNSS_epoch + 1;
        tor_s = time - time_last_GNSS;  % KF time interval
        time_last_GNSS = round(time,1);
        FIXFlag = GNSSObs(index_GNSS,5);
        GNSS_r_eb_e = GNSSObs(index_GNSS,2:4)';
        GNSS_v_eb_e =  GNSSObsVel(index_GNSSVel,5:7)';
        GNSS_Cov = max(GNSSObs(index_GNSS,7:9));
        %         [~,~,~,v_eb_n,est_C_b_n] = ECEF_to_NED(GNSS_r_eb_e,GNSS_v_eb_e,old_est_C_b_e);
        if strcmp(LC_KF_config.KFMethod,'ClassicKF')
            % Run Integration Kalman filter
            %             debug = [debug;time_last_GNSS,GNSS_r_eb_e',GNSS_v_eb_e',est_r_eb_e',est_v_eb_e',CTM_to_Euler(est_C_b_n')'...
            %                 .* 180/pi,est_IMU_bias',meas_f_ib_b_',norm(GNSS_v_eb_e),Sensor_];%Debug ,CalHeading(old_est_r_eb_e,GNSS_r_eb_e)
            %             if epoch>= 500 %&& FIXFlag == 1
            %                 %                 GNSS_index_old = find(round(GNSSObs(:,1),1)==round(time,1));
            %                 GNSS_index_fix = [GNSS_index_fix;index_GNSS,Sensor_];
            %                 if length(GNSS_index_fix) >10
            %                     if ~isnan(index_GNSS)
            %                         az_ = abs(CalHeading(GNSSObs(GNSS_index_fix(end-1,1),(2:4)),GNSSObs(GNSS_index_fix(end,1),(2:4)))-CalHeading(GNSSObs(GNSS_index_fix(end-5,1),(2:4)),GNSSObs(GNSS_index_fix(end-1,1),(2:4))));
            %                         if az_ > 300
            %                             az_ = 360 - az_ ;
            %                         end
            %                         if az_ >18 && Sensor_ >4 && GNSS_index_fix(end-5,2) > 2
            %                             %                             index_GNSS
            %                             GNSS_index_fix=GNSS_index_fix(1:end-1,:);
            %                             FIXFlag = 5;
            %                         end
            %                     end
            %                 end
            %             end
            % Adaptative KF
            if (abs(GNSS_Cov) >= 2.2) && FIXFlag == 2
                FIXFlag = 3;
            end
            %             if (abs(norm(GNSS_v_eb_e)) < 3 || Sensor_ < 3) && FIXFlag == 2
            %                 FIXFlag = 4;
            %             end
            %             if (abs(norm(GNSS_v_eb_e)) - Sensor_) > 1
            %                 FIXFlag = 5;
            %             end
            [est_C_b_e,est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix] =...
                LC_KF_Epoch(GNSS_r_eb_e,GNSS_v_eb_e,tor_s,est_C_b_e,...
                est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix,meas_f_ib_b,est_L_b,LC_KF_config,FIXFlag); %posvel KF
            %             [est_C_b_e,est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix] =...
            %                 LC_KFPOS_Epoch(GNSS_r_eb_e,GNSS_v_eb_e,tor_s,est_C_b_e,...
            %                 est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix,meas_f_ib_b,est_L_b,LC_KF_config,FIXFlag);
            %                 %only position KF
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
        if ~strcmp(LC_KF_config.KFMethod,'ClassicKF')
            out_MeasurementNoise_SD(index_GNSS,:)=[GNSSObs(index_GNSS,2:4),sqrt(diag(R_matrix(1:no_GNSS_meas,1:no_GNSS_meas)))];
        end
        lcflag = 1;
    elseif ((time - time_last_GNSS) >= GNSS_config.epoch_interval &&  isempty(index_GNSS) && ~isempty(index_GNSSVel))
        GNSS_epoch = GNSS_epoch + 1;
        tor_s = time - time_last_GNSS;  % KF time interval
        time_last_GNSS = round(time,1);
        GNSS_v_eb_e =  GNSSObsVel(index_GNSSVel,5:7)';
        %         debug = [debug;time_last_GNSS,est_r_eb_e',GNSS_v_eb_e',est_r_eb_e',est_v_eb_e',CTM_to_Euler(est_C_b_n')'...
        %             .* 180/pi,est_IMU_bias',meas_f_ib_b_',norm(GNSS_v_eb_e),Sensor_];%Debug CalHeading(old_est_r_eb_e,GNSS_r_eb_e)
        [est_C_b_e,est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix] =...
            LC_VEL_KF_Epoch(GNSS_v_eb_e,tor_s,est_C_b_e,...
            est_v_eb_e,est_r_eb_e,est_IMU_bias,P_matrix,meas_f_ib_b,est_L_b,LC_KF_config,LC_KF_config.vel_meas_SD^2 );
        lcflag = 2;
        %     elseif (((time - time_last_GNSS) >= GNSS_config.epoch_interval) &&  isempty(index_GNSS) && isempty(index_GNSSVel) &&  ~isempty(Sensor_)) %&& CarSpeed_ == 0
        %         tor_s = time - time_last_GNSS;
        %         %         %         %         if norm(GNSS_v_eb_e) < 1
        %         %         %                     [L_b,lambda_b,h_b,v_eb_n,est_C_b_n] = ECEF_to_NED(est_r_eb_e,est_v_eb_e,est_C_b_e);
        %         %         %         %             Euler_ = CTM_to_Euler(est_C_b_n');
        %         %         %         %             YawAngle_ = Euler_(3);
        %         %         %         %         else
        %         %         %                     YawAngle_ = GNSSYaw;
        %         %         %         %         end
        %         %         %                 CarSpeedNED = [CarSpeed_ * cos(YawAngle_);CarSpeed_ * sin(YawAngle_);0];
        %         %         %                 [~,CarVel,CarC_b_e] = NED_to_ECEF(L_b,lambda_b,h_b,CarSpeedNED,est_C_b_n);
        %         %         %                 CarVel = GNSS_v_eb_e .* CarSpeed_ /norm(GNSS_v_eb_e);
        %         %         %                 GNSS_v_eb_e = [CarSpeed_,0,0];
        %         %         %                 [est_C_b_e,est_v_eb_e,est_r_eb_e,est_IMU_bias,P_NED_matrix] =...
        %         %         %                     LC_LIMIT_KF_Epoch(tor_s,est_C_b_e,...
        %         %         %                     est_v_eb_e,est_r_eb_e,est_IMU_bias,P_NED_matrix,meas_f_ib_b,meas_omega_ib_b,LC_KF_config);
        %         lcflag = 3;
    end % if time
    
    %% Convert navigation solution to NED
    [est_L_b,est_lambda_b,~,~,est_C_b_n] =...
        ECEF_to_NED(est_r_eb_e,est_v_eb_e,est_C_b_e);
    
    % Generate output profile record
    out_profile(epoch,1) = time;
    out_profile(epoch,2:4)=(est_r_eb_e+est_C_b_e*L_ba_b)';%Position of antenna
    out_profile(epoch,5:7) = (est_v_eb_e+est_C_b_e*( Skew_symmetric(meas_omega_ib_b)*L_ba_b))';%Antenna velocity
    out_profile(epoch,8:10) = CTM_to_Euler(est_C_b_n')' .* 180/pi;
    Rotation=[   -sin(est_L_b)*cos(est_lambda_b),-sin(est_L_b)*sin(est_lambda_b),cos(est_L_b);
        -sin(est_lambda_b),cos(est_lambda_b),0;
        cos(est_L_b)*cos(est_lambda_b),cos(est_L_b)*sin(est_lambda_b),sin(est_L_b)];
    NEU=Rotation*(out_profile(epoch,2:4)-STA.STA(1).Coor(1:3))';
    out_profile(epoch,11:13)=[NEU(2),NEU(1),NEU(3)]';
    out_profile(epoch,14) = lcflag;
    % Reset old values
    old_time = time;
    old_est_r_eb_e = est_r_eb_e;
    old_est_v_eb_e = est_v_eb_e;
    old_est_C_b_e = est_C_b_e;
end %epoch

% Complete progress bar
fprintf(strcat(rewind,bars,'\n'));

% Ends