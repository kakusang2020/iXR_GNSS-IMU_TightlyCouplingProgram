clear;
addpath('..\Lib');
addpath('..\1_ReadIMU\Mechanization\lib\rotation');
addpath('F:\OnlyINS\0609');
GNSSpos=csvread('0609uboxENU.csv');
GNSSvel=csvread('0609GNSSvelocity.csv');
GNSScompass=csvread('rtk_all_kai.csv');
IMU=csvread('0609G370imu.csv');
CarSpeed=csvread('0609Carspeed.csv');
uboxcog=csvread('uboxFCOG.csv');
load('TestList.mat');
load('KFCog.mat');
stime=284042;att=[0;0;(-44.8050)*pi/180];vel=[0;0;0];initpos=[-101.246806700000;136.772878400000;-20.3387757400000];etime=286638;L_ba_b=[-1;0;-0.7];%286660
heightlist=[0,-20];time_last_GNSS = round(stime,1);speedvech=0;heading=att(3);speedtest=[];headingCheckList=[];debug=[];a=0;gyrobias=[];oldgyrobias_=0;TestList2=[];
biasupdatetime=0;headingErr_=0;updatetime=stime;headingold=stime;

%% G370 imu
IMUData_ = IMU;
IMUData_(:,5:7) = IMUData_(:,5:7)./180*pi;
IMUData_(:,2:7) = IMUData_(:,2:7) - mean(IMUData_(100:2000,2:7)); %remove bias
IMUData(:,1) =  IMUData_(:,1);
IMUData(:,2) = -IMUData_(:,2);
IMUData(:,3) = -IMUData_(:,3);
IMUData(:,4) = IMUData_(:,4) - 9.7978;
IMUData(:,5) = -IMUData_(:,5);
IMUData(:,6) = -IMUData_(:,6);
IMUData(:,7) = IMUData_(:,7);
%% Estelle imu body frame R-F-D to F-R-D 前右下 对应ned
% IMUData_ = IMU;
% IMUData_(:,2:4) = IMUData_(:,2:4) ;%.* 9.7978
% IMUData_(:,5:7) = IMUData_(:,5:7)./180*pi;%./180*pi;
% IMUData_(:,2:7) = IMUData_(:,2:7) - mean(IMUData_(100:4000,2:7)); %remove bias
% IMUData(:,1) = IMUData_(:,1) ;%+18
% IMUData(:,3) = IMUData_(:,2);
% IMUData(:,2) = IMUData_(:,3);
% IMUData(:,4) = -IMUData_(:,4) -9.8;
% IMUData(:,6) = IMUData_(:,5);
% IMUData(:,5) = IMUData_(:,6);
% IMUData(:,7) = -IMUData_(:,7);
%  plotEuler(IMUData,0);

%% Main Loop
for k =find(round(IMUData(:,1),2)==round(stime,2)):find(round(IMUData(:,1),2)==round(etime,2))
    a=a+1;
    dt = IMUData(k,1)-IMUData(k-1,1);
    gnsspos=GNSSpos(round(GNSSpos(:,1),1) == IMUData(k,1),2:4);
    gnssvel=GNSSvel(round(GNSSvel(:,1),1) == IMUData(k,1),13);
    gnsscompass=GNSScompass(round(GNSScompass(:,1),1) == IMUData(k,1),9);% Back ward
    carSpeed=CarSpeed(round(CarSpeed(:,1),2) == round(IMUData(k,1),2),1:end);
    cog=uboxcog(round(GNSScompass(:,1),1) == IMUData(k,1),2);
    headingErr=TestList(round(TestList(:,1),1) == IMUData(k,1),5)/180*pi;
    KFHeading=data3(round(data3(:,1),1) == IMUData(k,1),2)/180*pi;
%     if ~isempty(headingErr)
%         headingErr_=headingErr;
%     end
    if ~isempty(KFHeading)
        heading=KFHeading;
        headingold=heading;
    else
        heading=headingold;
    end
    
%     if ~isempty(gnsscompass)
%         if heading>pi
%             heading=heading-2*pi;
%         elseif heading<-pi
%             heading=heading+2*pi;
%         end
% %                 if abs(gnsscompass-heading)<300
% %                     TestList2=[TestList2;IMUData(k,1),gnsscompass,gnssvel*180/pi,heading*180/pi,gnsscompass/180*pi-heading,0];
% %                     if length(TestList)>300 && abs(TestList(end,4))<1 && std(TestList(end-50:end,4))<0.5
% %                         gyrobias=median(TestList(end-50:end,4));
% %                         oldgyrobias_=gyrobias/(-TestList(end-300,1)+TestList(end,1));
% %                         TestList(end,5)=oldgyrobias_;
% %                         if std(TestList(end-50:end,2))<0.06 && rem(length(TestList),600)<1
% %                             heading=mean(TestList(end-50:end,2))/180*pi;
% %                         end
% %                     end
% %                 end
%     end
    
    if ~isempty(gnsscompass) && ~isempty(cog)
        headingCheckList=[headingCheckList;IMUData(k,1),gnsscompass,cog,heading*180/pi];
    end
    
    if ~isempty(gnsspos)   && rem(k-13585,3000)<1%&&  ~isempty(gnsscompass) 13585  5324
        x(1:3)=gnsspos';
    elseif  ~isempty(carSpeed) %&& IMUData(k,1)-time_last_GNSS>=settings.updatefreq%WSS update isempty(gnssvel) &&
        y = [carSpeed(2)*sin(heading),carSpeed(2)*cos(heading),0];
        dt_=IMUData(k,1)-updatetime;
        x(1:3)=x(1:3)+y*dt_;
        updatetime=IMUData(k,1);
    else
    end
    outdata.x(a,:) = [IMUData(k,1),x];
end

%% plot
plot(outdata.x(:,2),outdata.x(:,3));
hold on
plot(GNSSpos(:,2),GNSSpos(:,3));
%% Compare
ubox=csvread('0609uboxB.csv');
ENUPos2 = ECEF2ENUPos(ubox);
Compar2file(outdata.x,ENUPos2);