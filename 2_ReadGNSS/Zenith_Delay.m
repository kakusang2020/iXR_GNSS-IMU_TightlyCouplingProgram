function [ZHD,ZWD]=Zenith_Delay(B,H,DOY)

%Calculate the Zenith Delay of tropsohere by using the EGNOS Model

%Inputs:
%  B     latitude(degree)
%  H     Height(m)
%  DOY   day of year

%Output
%  ZHD   Zenith Hydro Delay(m)
%  ZWD   Zenith Wet Delay(m)

% Written by: Zhang Yize

if B>0
    doymin=28;
else
    doymin=211;
end

k=cos(2*pi*(DOY-doymin)/365.25);
B=abs(B);
if abs(B)<15
    P=1013.25-0*k;
    beta=6.3-0*k;
    T=299.65-0*k;
    e=26.31-0*k;
    ranta=2.77-0*k;
elseif abs(B)<30
    m=(B-15)/15;
    P=1013.25+m*(1017.25-1013.25)-(0+m*(-3.75-0))*k;
    beta=6.3+m*(6.05-6.30)-(0+m*(0.25-0))*k;
    T=299.65+m*(294.15-299.65)-(0+m*(7.00-0))*k;
    e=26.31+m*(21.79-26.31)-(0+m*(8.85-0.0))*k;
    ranta=2.77+m*(3.15-2.77)-(0+m*(0.33-0.0))*k;
elseif abs(B)<45
    m=(B-30)/15;
    P=1017.25+m*(1015.75-1017.25)-(-3.75+m*(-2.25+3.75))*k;
    beta=6.05+m*(5.58-6.05)-(0.25+m*(0.32-0.25))*k;
    T=294.15+m*(283.15-294.15)-(7.00+m*(11.00-7.00))*k;
    e=21.79+m*(11.66-21.79)-(8.85+m*(7.24-8.85))*k;
    ranta=3.15+m*(2.57-3.15)-(0.33+m*(0.46-0.33))*k;
elseif abs(B)<60
    m=(B-45)/15;
    P=1015.75+m*(1011.75-1015.75)-(-2.25+m*(-1.75+2.25))*k;
    beta=5.58+m*(5.39-5.58)-(0.32+m*(0.81-0.32))*k;
    T=283.15+m*(272.15-283.15)-(11.00+m*(15.00-11.00))*k;
    e=11.66+m*(6.78-11.66)-(7.24+m*(5.36-7.24))*k;
    ranta=2.57+m*(1.81-2.57)-(0.46+m*(0.74-0.46))*k;
elseif abs(B)<75
    m=(B-60)/15;
    P=1011.75+m*(1013.00-1011.75)-(-1.75+m*(-0.50+1.75))*k;
    beta=5.39+m*(4.53-5.39)-(0.81+m*(0.62-0.81))*k;
    T=272.15+m*(263.65-272.15)-(15.00+m*(14.50-15.00))*k;
    e=6.78+m*(4.11-6.78)-(5.36+m*(3.39-5.36))*k;
    ranta=1.81+m*(1.55-1.81)-(0.74+m*(0.33-0.74))*k;
else    
    P=1013.00+0.5*k;
    beta=4.53-0.62*k;
    T=263.65-14.5*k;
    e=4.11-3.39*k;
    ranta=1.55-0.33*k;
end

ZHD=(77.604e-6)*287.054*P/9.784;   %Sea level zenith dry delay
ZWD=0.382*287.054/(9.784*(ranta+1)-beta/1000*287.054)*e/T;   %Sea level zenith wet delay
ZHD=ZHD*(1-beta/1000*H/T)^(9.80665/287.054/(beta/1000));  %Zenith dry delay
ZWD=ZWD*(1-beta/1000*H/T)^((ranta+1)*9.80665/287.054/(beta/1000)-1);  %Zenith wet delay
end