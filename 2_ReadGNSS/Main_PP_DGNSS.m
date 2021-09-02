%% Read RINEX
clc,clear
addpath('../Lib/GNSS_Tool_Lib');
addpath('../Data/20210412');
% First time read raw data
global ObsHead
global ObsData
global NavData
global ObsBaseData
% baseData=generateGnssData('base.obs','base.nav');
% ObsBaseData=baseData.obsData;
% gnssData = generateGnssData('rover.obs','base.nav');
% ObsData = gnssData.obsData;
% NavData = gnssData.navData;
% ObsHead = gnssData.headerData.obsHeader;

% Already read and saved the data
% global ObsHead
% global ObsData
% global NavData
load('ObsData0412.mat');
load('ObsBaseData0412.mat');
load('ObsHead0412.mat');
load('NavData0412.mat');

BaseCoor=[-3961905.0100,3348993.7800,3698211.8200];
ApproCoor = [-3961909.5034  ,3348998.9979  ,3698217.0898 ];   %init receiver position
ApproCoorV_ = [0,	0,	0]; %init receiver velocity

c = 299792458;   % Velocity of light
lambda1 = 0.1902936728;
F1 = 1575420000.0; %Frequency
B1 = 1561098000.0;
C1C=1; % From RINEX 3.02
D1C=3;
S1C=4;
D2L=7;
Epoch_Num=size(ObsData,2);
Epoch_DGNSSNum=size(ObsBaseData,2);
CorrectionDGNSS=[];
rec_clk=0;  %receiver clock
rec_clk_rate = 0; %receiver clock rate
Epoch_Used=[];  %The epoch be used
Obs_Coor=[];  %Receiver in each epoch
Obs_CoorV_=[]; %Receiver in each epoch
GDOP=[];       %GDOP in each epoch
PDOP=[];       %PDOP in each epoch
Rec_Clk=[];    %GPS clock difference
Rec_Clk_rate=[]; %GPS clock difference rate
GNSSTCData = [];%Data stroe for Tight coupling
%% DGNSS
for i=1:Epoch_DGNSSNum
    for j = 1:ObsBaseData(i).obsInfo.nSat
        if ~isempty(strfind(ObsBaseData(i).svObs(j).constellation,'GPS')) || ~isempty(strfind(ObsBaseData(i).svObs(j).constellation, 'QZSS'))
            if strcmp(ObsBaseData(i).svObs(j).constellation,'GPS') && ~isnan(ObsBaseData(i).svObs(j).measurements(ObsHead.obsType.typeIndexGPS.C1C))
                Range = ObsBaseData(i).svObs(j).measurements(ObsHead.obsType.typeIndexGPS.C1C) ;
                CorrectionDGNSS=[CorrectionDGNSS;ObsBaseData(i).time.GPST,...
                    ObsBaseData(i).svObs(j).svPRN,norm(BaseCoor-ObsBaseData(i).svObs(j).svPosition)-Range];
            elseif strcmp(ObsBaseData(i).svObs(j).constellation,'QZSS') && ~isnan(ObsBaseData(i).svObs(j).measurements(ObsHead.obsType.typeIndexGPS.C1C))
                Range = ObsBaseData(i).svObs(j).measurements(ObsHead.obsType.typeIndexQZSS.C1C) ; %-7.5?
                k=1;  %weight
                CorrectionDGNSS=[CorrectionDGNSS;ObsBaseData(i).time.GPST,...
                    ObsBaseData(i).svObs(j).svPRN,norm(BaseCoor-ObsBaseData(i).svObs(j).svPosition)-Range];
            else
                continue;
            end
            
        else
            continue
        end
    end
end
%% main loop
for i=1:Epoch_Num
    i
    if ObsData(i).obsInfo.epochFlag ~=0 || ObsData(i).obsInfo.nSat <4
        continue;
    end
    if i>=2641
        aa=1;
    end
    %     dx=ones(4,1);
    %     while max(abs(dx(1:3)))>=0.001
    %    Usually not necessary when less than 1km between Obs_Coor and ApproCoor
    while (1)
        A=[];
        L=[];
        P=[];
        L_noclk = [];
        
        AV_=[];
        LV_=[];
        PV_=[];
        
        [Lat,Lon,Height]=XYZ2BLH(ApproCoor(1),ApproCoor(2),ApproCoor(3));  %ECEF to Lat Lon Height
        Rota=[-sind(Lat)*cosd(Lon) -sind(Lon) cosd(Lat)*cosd(Lon);-sind(Lat)*sind(Lon) cosd(Lon) cosd(Lat)*sind(Lon);cosd(Lat) 0 sind(Lat)];
        %Transformation matrix of space geodetic Cartesian coordinate system to station center horizontal coordinate system
        [ZHD,ZWD]=Zenith_Delay(Lat,Height,ObsHead.DOY); %Calculation of tropospheric zenith delay
        data_ = zeros(ObsData(i).obsInfo.nSat,16);
        for j = 1:ObsData(i).obsInfo.nSat
            if ~isempty(strfind(ObsData(i).svObs(j).constellation,'GPS')) || ~isempty(strfind(ObsData(i).svObs(j).constellation, 'QZSS'))
                %Satelate position
                %j
                [Xs,Ys,Zs,sat_clk,rela,s, rate_clock, Xsvel, Ysvel, Zsvel, delta_tsv_L1pie]=Cal_Sat_Pos(ObsData(i).time(1).GPSWeek,ObsData(i).time(1).GPST-rec_clk,ObsData(i).svObs(j),ApproCoor);
                Sat_xyz=([Xs ,Ys ,Zs]-ApproCoor)*Rota; %The coordinates of satellite in the station center horizontal coordinate system
                ele=asind(Sat_xyz(3)/sqrt(sum(Sat_xyz.^2))); %elevation
                %                                 if ele<10   %Mask angle
                %                                     %i %5087
                %                                     continue;
                %                                 end
                %Signal freq pseudorange
                if strcmp(ObsData(i).svObs(j).constellation,'GPS') && ~isnan(ObsData(i).svObs(j).measurements(ObsHead.obsType.typeIndexGPS.C1C))
                    Range = ObsData(i).svObs(j).measurements(ObsHead.obsType.typeIndexGPS.C1C) ; %-7.5?
                    k=1;  %weight
                elseif strcmp(ObsData(i).svObs(j).constellation,'QZSS') && ~isnan(ObsData(i).svObs(j).measurements(ObsHead.obsType.typeIndexGPS.C1C))
                    Range = ObsData(i).svObs(j).measurements(ObsHead.obsType.typeIndexQZSS.C1C) ; %-7.5?
                    k=1;  %weight
                else
                    continue;
                end
                A=[A;(ApproCoor-[Xs ,Ys ,Zs])/s 1]; AV_ = A;%Direction cosine
                %Tropospheric delay in the direction of oblique path is calculated according to station coordinates and altitude angle
                Trop=Neill_Map(ZHD,ZWD,ele,Lat,Height,ObsHead.DOY); %Trop delay
                L=[L;Range-s-Trop+c*sat_clk+c*rela-c*rec_clk];
                %%%%%%Store data%%%%%%%%
                rate = -1.0 * ObsData(i).svObs(j).measurements(D1C) * c / F1;
                L_noclk=[L_noclk;Range-Trop+c*sat_clk+c*rela];
                
                LV_ = [LV_; rate + ( AV_(end,1) .* (Xsvel-ApproCoorV_(1)) + AV_(end,2) .* (Ysvel-ApproCoorV_(2)) + ...
                    AV_(end,3) .* (Zsvel-ApproCoorV_(3))) - c * delta_tsv_L1pie];
                %rate_clock cow10
                data_(j,:) = [i, ObsData(i).time(1).GPSWeek, ObsData(i).time(1).GPST, ObsData(i).svObs(j).svPRN, ...
                    L_noclk(end), Xs,Ys,Zs, rate, rate_clock,Xsvel, Ysvel, Zsvel,ele, ObsData(i).svObs(j).measurements(S1C), LV_(end)];
                
                %%%%%%Store data end%%%%%%%%
                if ele>30
                    P=[P;k];
                else
                    P=[P;k*4*(sind(ele))^2];
                end
                PV_ = P;
            elseif isfield(ObsData(i).svObs(j),'BDS')
                continue;
            else
                continue
            end
        end
        
        
        vnum=1;
        while ~isempty(vnum)
            if size(A,1)<4  %When satellite less than 4, without calculation
                break;
            end
            N=A'*diag(P)*A;
            Q=inv(N);
            gdop=sqrt(trace(Q));
            pdop=sqrt(trace(Q(1:3,1:3)));
            %     if gdop>12
            %         continue;
            %     end
            dx=(Q*(A'*diag(P)*L))'; %Clock difference
            V=A*dx'-L;
            sigma0=sqrt(V'*diag(P)*V/(size(A,1)-4));
            vnum=find(abs(V.*sqrt(P))>2.5*sigma0);
            A(vnum,:)=[];
            L(vnum)=[];
            P(vnum)=[];
        end
        
        vnum_ = 1;
        while ~isempty(vnum_)
            if size(AV_,1)<4  %When satellite less than 4, without calculation
                break;
            end
            N=AV_'*diag(PV_)*AV_;
            Q=inv(N);
            dxV_ =(Q*(AV_'*diag(PV_)*LV_))'; %(Vx,Vy, Vz, clock rate)
            V=AV_*dxV_'-LV_;
            sigma0=sqrt(V'*diag(PV_)*V/(size(AV_,1)-4));
            vnum_=find(abs(V.*sqrt(PV_))>2.5*sigma0);
            AV_(vnum_,:)=[];
            LV_(vnum_)=[];
            PV_(vnum_)=[];
        end
        
        ApproCoor=ApproCoor+dx(1:3);
        ApproCoorV_ = ApproCoorV_ + dxV_(1:3);
        rec_clk=rec_clk+dx(4)/c;
        rec_clk_rate = dxV_(4);
        if all(abs(dx(:))<0.0001)
            break;
        end
    end
    %Epoch_Used=[Epoch_Used;i];
    %GDOP=[GDOP;gdop];
    %PDOP=[PDOP;pdop];
    GNSSTCData = [GNSSTCData; data_];
    Rec_Clk=[Rec_Clk;rec_clk,dx(4)];
    Rec_Clk_rate =[Rec_Clk_rate;rec_clk_rate,dxV_(4)];
    Obs_Coor=[Obs_Coor;ApproCoor];  %Position (m)
    Obs_CoorV_=[Obs_CoorV_;ApproCoorV_]; %Velocity in X, Y, Z; (m/s)
end
% plotCoor(Obs_Coor);