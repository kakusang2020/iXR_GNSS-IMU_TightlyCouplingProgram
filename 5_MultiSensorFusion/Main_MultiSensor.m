%% init
clear;
addpath('../Lib/GNSS_Tool_Lib');
addpath('../Lib/INS_Tool_Lib');
%% 1105
% stime=364534;etime=366367;att=[0;0;-2.326737];vel=[0;0;0];initpos=[-102.0682645;	137.5323719;	-20.29763465];L_ba_b=[0;0;-0.7];
%% 03023
% stime=187540;etime=189629;att=[0;0;(-49.5)/180*pi];vel=[0;0;0];initpos=[-102.5932344;	138.0176275;	-19.83667144];L_ba_b=[0;0;-0.7];
%% 0412
stime=111184;etime=113207; att= [0;0;(-49)/180*pi];vel=[0;0;0];initpos=[-102.369033600000;137.827406400000;-20.2645098500000];L_ba_b=[0;0;-0.7];
addpath('../Data/20210412');%('../Data/20210302/3th');%('../Data/20210609');
GNSSpos=csvread('RTKENU0412.csv');%('RTKENU03023.csv');
GNSSvel=csvread('velocity0412.csv');%('GNSSvel03023.csv');
% GNSScompass=csvread('rtk_all_kai.csv');
IMU=csvread('imu0412_epsonG370_BLD.csv');
CarSpeed=csvread('Sensor0412.csv');
ubox=csvread('ubox0412.csv');
heightlist=[0,-19.84,0];time_last_GNSS = round(stime,1);speedvech=0;heading=att(3);speedtest=[];headingCheckList=[];debug=[];a=0;gyrobias=[];oldgyrobias_=0;TestList=[];
airPrebias=135.5525;
meanHeight=mean(GNSSpos(1:100,4));
%speedhor_=27.2315;
% stime=84023.1;etime=84490;att=[0;0;130*pi/180];vel=[12.025;-8.871;0];%etime [-23682.1907300000,8967.05445800000,61.5578288700000]
% stime=453600;etime=456615;att=[0;0;-45.6*pi/180];vel=u[0;0;0];initpos=[137.302642800000;-102.060873900000;-20.2896912700000];%etime=98564.6;
%% 0609
% addpath('../Data/20210609');%('../Data/20210412');%('../Data/20210302/3th');%
% GNSSpos=csvread('0609RTKENUB.csv');%('RTKENU0412.csv');%('RTKENU03023.csv');
% GNSSvel=csvread('0609GNSSvelocity.csv');%('velocity0412.csv');%('GNSSvel03023.csv');
% % GNSScompass=csvread('rtk_all_kai.csv');
% IMU=csvread('0609G370imu.csv');
% CarSpeed=csvread('0609Sensor.csv');
% Ubox=csvread('0609uboxB.csv');
% % poslv=csvread('03023carspeed.csv');
% stime=284042;att=[0;0;(-44.8050)*pi/180];vel=[0;0;0];initpos=[-101.246806700000;136.772878400000;-20.3387757400000];etime=286638;L_ba_b=[-1;0;-0.7];%286660
% heightlist=[0,-19.84];time_last_GNSS = round(stime,1);speedvech=0;heading=att(3);speedtest=[];headingCheckList=[];debug=[];a=0;gyrobias=[];oldgyrobias_=0;TestList=[];%speedhor_=27.2315;
% biasupdatetime=0;
%% Estelle imu body frame R-F-D to F-R-D 前右下 对应ned
% IMUData_ = IMU;
% IMUData_(:,2:4) = IMUData_(:,2:4) ;%.* 9.7978
% IMUData_(:,5:7) = IMUData_(:,5:7)./180*pi;%./180*pi;
% IMUData_(:,2:7) = IMUData_(:,2:7) - mean(IMUData_(2000:6000,2:7)); %remove bias
% IMUData(:,1) = IMUData_(:,1) ;%+18
% IMUData(:,3) = IMUData_(:,2);
% IMUData(:,2) = IMUData_(:,3);
% IMUData(:,4) = -IMUData_(:,4) -9.8;
% IMUData(:,6) = IMUData_(:,5);
% IMUData(:,5) = IMUData_(:,6);
% IMUData(:,7) = -IMUData_(:,7);
%% G370 imu
IMUData_ = IMU;
IMUData_(:,5:7) = IMUData_(:,5:7)./180*pi;
IMUData_(:,2:7) = IMUData_(:,2:7) - mean(IMUData_(100:2000,2:7)); %remove bias
IMUData_(:,2:7) = IMUData_(:,2:7) - mean(IMUData_(100:2000,2:7)); %remove bias
IMUData(:,1) =  IMUData_(:,1);
IMUData(:,2) = -IMUData_(:,2);
IMUData(:,3) = -IMUData_(:,3);
IMUData(:,4) = IMUData_(:,4) - 9.7978;
IMUData(:,5) = -IMUData_(:,5);
IMUData(:,6) = -IMUData_(:,6);
IMUData(:,7) = IMUData_(:,7);
% plotEuler(IMUData,0);
settings = gnss_imu_local_tan_example_settings();
predic_div=0;
q = ch_eul2q(att);
x = [initpos;vel; q];
% Initialize the sensor bias estimate
delta_u = zeros(6, 1);
% Initialize the Kalman filter
[P, Q, ~, ~] = init_filter(settings);
%% Main Loop
for k =find(round(IMUData(:,1),2)==round(stime,2)):find(round(IMUData(:,1),2)==round(etime,2))
    a=a+1;
    dt = IMUData(k,1)-IMUData(k-1,1);
    gnsspos=GNSSpos(round(GNSSpos(:,1),1) == IMUData(k,1),2:4);
    gnssvel=GNSSvel(round(GNSSvel(:,1),1) == IMUData(k,1),3:5);
    %     gnsscompass=GNSScompass(round(GNSScompass(:,1),1) == IMUData(k,1),9);% Back ward
    Fixflag=GNSSpos(round(GNSSpos(:,1),1) == IMUData(k,1),5);
    nsat=GNSSpos(round(GNSSpos(:,1),1) == IMUData(k,1),6);
    carSpeed=CarSpeed(round(CarSpeed(:,1),2) == round(IMUData(k,1),2),1:end);
    carSpeedold=CarSpeed(find(round(CarSpeed(:,1),2) == round(IMUData(k,1),2))-1,1:end);
    
    %     poslvod=poslv(round(poslv(:,1),3) == round(IMUData(k,1),2),3);
    %     carSpeed(5)=poslvod;
    % 零偏状态反馈
    %% zupt
    %     if carSpeed(2) == 0
    %         zuptflag = 1;
    %     else
    %         zuptflag = 0;
    %     end
    %     if zuptflag == 1 %&& zuptflag==1%ZUPT
    %         gyrobias = [gyrobias;IMUData(k,7)];gyrobias_=0;
    %     end
    %     if length(gyrobias)>300 && zuptflag ==0
    %         %         gyrobias_ = sum(gyrobias)./length(gyrobias);gyrobias=[];
    %         gyrobias_ = median(gyrobias);gyrobias=[];zuptflag=1;
    %         oldgyrobias_=gyrobias_;
    %     elseif zuptflag ==0
    %         gyrobias=[];
    %%     end
    u_h = IMUData(k,2:7)' - delta_u(1:6);
    % 捷联惯导解算
    [x(1:3), x(4:6), x(7:10)] = ch_nav_equ_local_tan(x(1:3), x(4:6), x(7:10), u_h(1:3), u_h(4:6), dt, settings.gravity);
    predic_div = predic_div+1;
    if predic_div == 10
        [F, G] = state_space_model(x, u_h, dt*predic_div);
        P = F*P*F' + G*Q*G';
        predic_div = 0;
    end
    % 航向更新
    heading=heading+u_h(6)*dt;
    
    %%     if ~isempty(gnsscompass)
    %         if heading>pi
    %             heading=heading-2*pi;
    %         elseif heading<-pi
    %             heading=heading+2*pi;
    %         end
    %         if abs(gnsscompass-heading)<300
    %             TestList=[TestList;IMUData(k,1),gnsscompass,heading*180/pi,gnsscompass/180*pi-heading,0];
    %             if length(TestList)>300
    %                 gyrobias=median(TestList(end-300:end,4));
    %                 oldgyrobias_=gyrobias/(-TestList(1,1)+TestList(end,1));
    %                 TestList(end,5)=oldgyrobias_;
    %                 if std(TestList(end-50:end,2))<0.06 && rem(length(TestList),600)<1
    %                     heading=mean(TestList(end-50:end,2))/180*pi;
    %                 end
    %             end
    %         end
    %     end
    %
    %     if ~isempty(gnsscompass)
    %         headingCheckList=[headingCheckList;IMUData(k,1),gnsscompass,heading*180/pi];
    %     end
    %     if length(headingCheckList)>101 && mod(k,60*50)<2
    %         if std(headingCheckList(end-100:end,2))<0.02
    %             heading = median(headingCheckList(end-100:end,2))/180*pi;
    %         end
    %%     end
    
    %     % 垂直方向速度
    imuHeight=153.8*(carSpeed(8)+273.2)*(1-(carSpeed(7)/1013.25)^0.1902)+airPrebias;
    heightincrement=44300*(1- (carSpeed(7)/1015.25)^(1/5.256)) - 44300*(1- (carSpeedold(7)/1015.25)^(1/5.256));
    heightlist=[heightlist;IMUData(k,1),heightlist(end,2)+heightincrement,imuHeight]; %heightincrement list
    if length(heightlist)>100 && dt~=0
%         airPrebias=meanHeight-mean(heightlist(:,3));
        speedvech=(median(heightlist(end-20:end,3))-median(heightlist(end-100:end-80,3)))/dt/100;
    else
        speedvech=0;
    end
    
    %%     gnsscompass=rem(gnsscompass*0.0175+1*pi,2*pi);
    %         if ~isempty(gnsspos)  &&  ~isempty(gnssvel) &&  ~isempty(gnsscompass) && (gnsscompass>0.3 || gnsscompass<2*pi-0.3)%pos vel and att update
    %             y = [gnsspos';gnssvel';[0;0;(gnsscompass)]];
    %             H = [eye(3) zeros(3,3) zeros(3,9)
    %                 zeros(3,3) eye(3)  zeros(3,9)
    %                 zeros(3,6) eye(3)  zeros(3,6)];
    %             % Adaptive R
    %             if abs(gnsspos(3) - heightlist(end)) >5
    %                 settings.sigma_gpspos = 0.001/sqrt(3)*10 * abs(gnsspos(3) - heightlist(end));
    %             elseif Fixflag ==2
    %                 settings.sigma_gpspos = 0.001/sqrt(3)*10 ;
    %             elseif Fixflag ==1
    %                 settings.sigma_gpspos = 0.001/sqrt(3);
    %             end
    %             if Fixflag ==1 && abs(gnsspos(3) - heightlist(end)) <1 %remove air pressure bias
    %                 heightlist(end)=gnsspos(3);
    %             end
    %             if carSpeed(5)<3 %low speed correction
    %                 settings.sigma_gpspos = 0.001/sqrt(3)*500;
    %                 y(4:6)=y(4:6).*carSpeed(5)/norm(y(4:6));
    %             end
    %             %             if k>1        4000
    %             %                 if abs(x(1:3)-outdata.x(2:4,k-1))>carSpeed(2)*1.3
    %             %                 settings.sigma_gpspos = 0.001/sqrt(3)*500;
    %             %                 end
    %             %             end
    %             if norm(x(4:6))>carSpeed(5)*1.3
    %                 settings.sigma_gpsvel = 0.001/sqrt(3)*100;
    %             end
    %             R = [settings.sigma_gpspos^2*eye(3) zeros(3) zeros(3)
    %                 zeros(3) settings.sigma_gpsvel^2*eye(3) zeros(3)
    %                 zeros(3) zeros(3) settings.sigma_gpsatt^2*eye(3)];
    %             K=(P*H')/(H*P*H'+R);
    %             att=ch_q2eul(x(7:10));
    %             att(3)=rem(att(3)-y(9),2*pi);
    %             z_=y - [x(1:6);att];
    %             z_(9)=att(3);
    %             z = [zeros(9,1); delta_u] + K*z_;
    %             z_(9)
    %%             updateflag = 1;
    if ~isempty(gnsspos)  &&  ~isempty(gnssvel) % &&  ~isempty(gnsscompass)%vel and att update
        if 0%Fixflag ==2 && nsat <= 10 % WSS update
            y = [carSpeed(5)*sin(heading);carSpeed(5)*cos(heading);0];
            H = [zeros(3,3) eye(3) zeros(3,9)];
            R = settings.sigma_speed ^2*eye(3)*2;
            K=(P*H')/(H*P*H'+R);
            z = [zeros(9,1); delta_u] + K*(y - x(4:6));
            updateflag = 4;
        else
            y = [gnsspos';gnssvel'];
            H = [   eye(3) zeros(3,3) zeros(3,9)
                zeros(3,3) eye(3) zeros(3,9)];
            %% Adaptive R
            if abs(gnsspos(3) - heightlist(end,3)) >5 || abs(gnssvel(3)) > 1
                settings.sigma_gpspos = 0.001/sqrt(3)*10 * abs(gnsspos(3) - heightlist(end,2));
                settings.sigma_gpsvel=settings.sigma_gpsvel*abs(gnssvel(3));
            elseif Fixflag ==1
                settings.sigma_gpspos = 0.001/sqrt(3)*1 ;
            else
                settings.sigma_gpspos = 0.001/sqrt(3)*10;
            end
            if carSpeed(5)<3 %&& Fixflag ==2%low speed correction
                settings.sigma_gpspos = 0.001/sqrt(3)*50;
                y(4:6)=y(4:6).*carSpeed(5)/norm(y(4:6));
            end
            if Fixflag ==1 && abs(gnsspos(3) - heightlist(end,2)) <1 %remove air pressure bias
                heightlist(end,2)=gnsspos(3);
            end
            %% KF
            R = [settings.sigma_gpspos^2*eye(3) zeros(3)
                zeros(3) settings.sigma_gpsvel^2*eye(3)];
            K=(P*H')/(H*P*H'+R);
            z = [zeros(9,1); delta_u] + K*(y - x(1:6));
            updateflag = 2;
        end
    elseif isempty(gnsspos) && ~isempty(gnssvel) %Velocity update
        y = gnssvel';
        H = [zeros(3,3) eye(3) zeros(3,9)];
        R = settings.sigma_gpsvel^2*eye(3);
        K=(P*H')/(H*P*H'+R);
        z = [zeros(9,1); delta_u] + K*(y - x(4:6));
        updateflag = 3;
%     elseif  ~isempty(carSpeed) && IMUData(k,1)-time_last_GNSS>=settings.updatefreq%WSS update
%         y = [carSpeed(5)*sin(heading);carSpeed(5)*cos(heading);0];
%         H = [zeros(3,3) eye(3) zeros(3,9)];
%         R = settings.sigma_speed ^2*eye(3);
%         K=(P*H')/(H*P*H'+R);
%         z = [zeros(9,1); delta_u] + K*(y - x(4:6));
%         updateflag = 4;
    else
        updateflag = 0;
    end
    %% Correct the navigation states using current perturbation estimates.
    if updateflag ~= 0
        time_last_GNSS=IMUData(k,1);
        % 位置速度反馈
        x(1:6) = x(1:6) + z(1:6);
        % 失准角反馈到姿态
        q = x(7:10);
        q = ch_qmul(ch_rv2q(z(7:9)), q);
        x(7:10) = q;
        delta_u = z(10:15);  %bias
        %更新P 使用Joseph 形式，取代 (I-KH)*P, 这么数值运算更稳定
        I_KH = (eye(size(P,1))-K*H);
        P= I_KH*P*I_KH' + K*R*K';
    end
    % Save the data to the output data structure
    outdata.x(a,:) = [IMUData(k,1),x',updateflag];
    outdata.eul(a,:) = [IMUData(k,1),ch_q2eul(x(7:10))',heading];%,rem(outdata.eul(a,4)-outdata.eul(a,5),2*pi)
    outdata.diag_P(a,:) = diag(P);
    outdata.delta_u_h(a,:) = delta_u;
end
%% plot
plot(outdata.x(:,2),outdata.x(:,3));
hold on
plot(GNSSpos(:,2),GNSSpos(:,3));
%% Compare
% ubox=csvread(Ubox);
ENUPos2 = ECEF2ENUPos(ubox);
Compar2file(outdata.x,ENUPos2);