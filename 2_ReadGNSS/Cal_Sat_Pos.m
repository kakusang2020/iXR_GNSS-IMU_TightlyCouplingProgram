function [Xs ,Ys ,Zs ,sat_clk ,rela ,R,rate_clock, Xsvel, Ysvel, Zsvel, delta_tsv_L1pie]=Cal_Sat_Pos(GPSweek,GPSsec,svObs,ApproCoor)

% Calculate the Satellite Coordinate, satellite clock error, relativity
% error, the range between the satellite and the receiver

% Inputs:
%    GPSweek           GPS week
%    GPSsec            GPS second
%    PRN               Pseudo-Random Noise,satellite number
%    ApproCoor         Receiver position

% Outpus:
%    Xs,Ys,Zs                   Coordinate of the satellite
%    sat_clk                    clock error of the receiver
%    rela                         relativity error
%    R                            range between the satellite and the receiver
%    rate_clock               satellite rate_clock
%    Xsvel, Ysvel, Zsvel   Coordinate velocity of the satellite
%    delta_tsv_L1pie       Satellite clock error rate of change

% Written by: Zhang Yize
% Changed by: Guo Xiaoliang

global NavData

%  Constants
GM=3.986005e+14; %3.986004418e+14;      % universal gravitational param (m^3/s^2)
omgedote=7.292115e-5;  % 7.2921151467e-5 earth's rotation rate (rad/sec)
c=299792458;        % speed of light (m/s)

tkr=inf;
if ~isempty(strfind(svObs.constellation,'GPS'))
    svPRNStr = num2str(svObs.svPRN ,'GPS%03d');
    for j = 1 : length(NavData.GPS.(svPRNStr))
        
        t=(GPSweek-NavData.GPS.(svPRNStr)(j).weekno)*604800+GPSsec-NavData.GPS.(svPRNStr)(j).toe;
        if abs(t)<abs(tkr)%-60   % in case of 59min44sec
            tkr=t;  %The time difference between the time when the receiver receives the signal and the reference time
            tempNav=NavData.GPS.(svPRNStr)(j);
        elseif abs(t)>abs(tkr)+7000   % in case of 59min44sec
            break;
        end
        
    end
end

if ~isempty(strfind(svObs.constellation,'QZSS'))
    svPRNStr = num2str(svObs.svPRN ,'QZSS%03d');
    for k = 1 : length(NavData.QZSS.(svPRNStr))
        
        t=(GPSweek-NavData.QZSS.(svPRNStr)(k).weekno)*604800+GPSsec-NavData.QZSS.(svPRNStr)(k).toe;
        if abs(t)<abs(tkr)%-60   % in case of 59min44sec
            tkr=t;  %The time difference between the time when the receiver receives the signal and the reference time
            tempNav=NavData.QZSS.(svPRNStr)(k);
        elseif abs(t)>abs(tkr)+7000   % in case of 59min44sec
            break;
        end
        
    end
end

if (abs(tkr))>7210
    Xs=nan;
    Ys=nan;
    Zs=nan;
    sat_clk=nan;
    rela=nan;
    R=nan;
    return
    %     tkr
end
e=tempNav.ecc;
A=(tempNav.roota)^2;
n0=sqrt(GM/A^3);
n=n0+tempNav.deltan; % Corrected mean motion (rad/s)
t1=0.075; %Time required for signal propagation,0.075s
% t1=0;
dt=1;
while abs(dt)>1e-10
    tk=tkr-t1;  %The time difference between the time when the satellite transmits the signal and the reference time
    Mk=tempNav.M0+n*tk;   % Mean anomaly (rad/s)
    Ek0=Mk;dEk=1;
    while dEk>1e-9
        Ek=Mk+e*sin(Ek0);
        dEk=abs(Ek-Ek0);
        Ek0=Ek;
    end
    vk=2*atan(tan(Ek/2)*sqrt((1+e)/(1-e))); % True anom (rad)
    if(vk<0)
        vk=vk+2*pi;
    end
    PHI_k=vk+tempNav.omega;   % Argument of latitude
    
    % Second Harmonic Perturbations
    duk=tempNav.cus*sin(2*PHI_k)+tempNav.cuc*cos(2*PHI_k);  % Argument of Lat correction
    drk=tempNav.crs*sin(2*PHI_k)+tempNav.crc*cos(2*PHI_k);  % Radius correction
    dik=tempNav.cis*sin(2*PHI_k)+tempNav.cic*cos(2*PHI_k);  % Inclination correction
    
    uk=PHI_k+duk;                    % Corr. arg of lat
    r=A*(1-e*cos(Ek))+drk;               % Corrected radius
    ik=tempNav.i0+dik+tempNav.idot*tk;   % Corrected inclination
    
    % Positons in orbital plane
    x=r*cos(uk);
    y=r*sin(uk);
    
    Omega=tempNav.Omega0+(tempNav.Omegadot-omgedote)*tk-omgedote*tempNav.toe;
    
    % ECEF coordinates
    Xs=x*cos(Omega)-y*cos(ik)*sin(Omega);
    Ys=x*sin(Omega)+y*cos(ik)*cos(Omega);
    Zs=y*sin(ik);
    sat_clk=tempNav.af0+tk*tempNav.af1+tk^2*tempNav.af2 - tempNav.tgd1;
    
    %clock-error correction with theory of relativity%
    F=-4.442807633*1.0e-10;
    rela=F*e*sqrt(A)*sin(Ek);  %unit is second
    
    %Satellite position with earth rotation correction
    Rotation=[cos(omgedote*t1),sin(omgedote*t1),0;-sin(omgedote*t1),cos(omgedote*t1),0;0,0,1];
    %     Rotation2=[1,sin(omgedote*t1),0;-sin(omgedote*t1),1,0;0,0,1];
    SatCoor=Rotation*[Xs;Ys;Zs];
    Xs=SatCoor(1);Ys=SatCoor(2);Zs=SatCoor(3);
    R=sqrt(sum((ApproCoor-SatCoor').^2));  %Distance between ApproCoor and SatCoor
    
    %delta_tsv_L1pie = tempNav.af1 + 2 * tempNav.af2 + rela;
    delta_tsv_L1pie = tempNav.af1 + 2 * tempNav.af2 ;
    %Satelite velocity
    mk=n;                   %Horizontal proximal angle
    ek=mk/(1-tempNav.ecc*cos(Ek));             %Near point angle
    Wk=sqrt(1-tempNav.ecc^2)*ek/(1-tempNav.ecc*cos(Ek));           %Angular distance of ascending intersection
    Uuk=2*Wk*(tempNav.cus*cos(2*PHI_k)-tempNav.cuc*sin(2*PHI_k));       %Perturbation correction term
    Rrk=2*Wk*(tempNav.crs*cos(2*PHI_k)-tempNav.crc*sin(2*PHI_k));
    Iik=2*Wk*(tempNav.cis*cos(2*PHI_k)-tempNav.cic*sin(2*PHI_k));
    Uk=Wk+Uuk;                                      %After correction
    Rk=(tempNav.roota^2)*tempNav.ecc*ek*sin(Ek)+Rrk;
    Ik=tempNav.idot+Iik;
    WK=tempNav.Omegadot-7.2921151467e-5;           %Right ascension of ascending node
    Xxk=Rk*cos(uk)-r*Uk*sin(uk);
    Yyk=Rk*sin(uk)+r*Uk*cos(uk);
    
    Xsvel=-Ys*WK-(Yyk*cos(ik)-Zs*Ik)*sin(Omega)+Xxk*cos(Omega);
    Ysvel=Xs*WK+(Yyk*cos(ik)-Zs*Ik)*cos(Omega)+Xxk*sin(Omega);
    Zsvel=Yyk*sin(ik)+y*Ik*cos(ik);
    
    rate_clock = tempNav.af1 + tempNav.af2 * t;%Rate of Satellite clock=a1+a2*(Observation time - ephemeris reference time)
    
    %rate_clock
    
    t2=R/c;
    dt=t2-t1;
    t1=t2;
end

clear t1 t2 dt
end