%% init
clear;
% addpath('\0531');addpath('D:\hfss model\MatlabCode\Tightly coupling program\iXR_GNSS-IMU_TightlyCouplingProgram-master\3_TightlyCoupling');
addpath(genpath('F:\ExperienceData\IMUPaper\nav_matlab-master (1)\nav_matlab-master\lib'));
addpath('F:\ExperienceData\IMUPaper\nav_matlab-master (1)\nav_matlab-master\lib\rotation');
addpath('C:\Users\denshi\Desktop\OnlyINS\0531');
GNSSpos=csvread('ubox2neu.csv');
IMU=csvread('Estelle05312.csv');
CarSpeed=csvread('Estelle05312Sensor.csv');
% stime=84023.1;etime=84490;att=[0;0;130*pi/180];vel=[12.025;-8.871;0];%etime [-23682.1907300000,8967.05445800000,61.5578288700000]
stime=98191;etime=98566.6;att=[0;0;-49*pi/180];vel=[0;0;0];initpos=[-22552.8770200000,7596.78441900000,20.7294042100000];speed=27.2315;
test=[0];speedvech=0;heading=0;result=[];DRpos=[0,0,0];
%% Estelle imu body frame R-F-D to F-R-D
IMUData_ = IMU;
IMUData_(:,2:4) = IMUData_(:,2:4) ;%.* 9.7978
IMUData_(:,5:7) = IMUData_(:,5:7);%./180*pi;%./180*pi;
IMUData_(:,2:7) = IMUData_(:,2:7) - mean(IMUData_(3000:13000,2:7)); %remove bias
IMUData(:,1) = IMUData_(:,1) ;%+18
IMUData(:,3) = IMUData_(:,2);
IMUData(:,2) = IMUData_(:,3);
IMUData(:,4) = -IMUData_(:,4) -9.8;
IMUData(:,6) = IMUData_(:,5);
IMUData(:,5) = IMUData_(:,6);
IMUData(:,7) = -IMUData_(:,7);
%%
for k =find(round(IMUData(:,1),2)==round(stime,2))+1:find(round(IMUData(:,1),2)==round(etime,2))
    dt=IMUData(k,1)-IMUData(k-1,1);
    heading=heading+att(3)+IMUData(k,7)*dt;att(3)=0;
%     speed=speed+IMUData(k,2)*dt;
    speed=CarSpeed(k,2);
    DRpos=DRpos+initpos+[cos(heading)*speed,sin(heading)*speed,0].*dt;initpos=[0,0,0];
    result=[result;IMUData(k,1),DRpos,heading*180/pi];
end
%%
plot(result(:,3),result(:,2));
hold on
plot(GNSSpos(:,3),GNSSpos(:,2));