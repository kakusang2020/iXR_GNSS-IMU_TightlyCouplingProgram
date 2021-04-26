function [lat,lon,h]=XYZ2BLH(X,Y,Z)

%��X��Y��Zת����B��L��H

%   X��Y��Z   ��Ŀռ�ֱ������X��Y��Z,��λΪm
%   B��L��H   ���B��L��H,B��L�ĵ�λΪ�ȣ�H�ĵ�λΪm

% Written by: Zhang Yize

a=6378137;e2=0.00669438002290; %WGS84������Ԫ��
elat=1.e-12;
eht=1.e-5;
p=sqrt(X.*X+Y.*Y);
lat=atan2(Z,p./(1-e2));
h=0;
dh=1;
dlat=1;
while sum(dlat>elat) || sum(dh>eht)
  lat0=lat;
  h0=h;
  v=a./sqrt(1-e2.*sin(lat).*sin(lat));
  h=p.*cos(lat)+Z.*sin(lat)-(a*a)./v;  % Bowring formula
  lat=atan2(Z, p.*(1-e2.*v./(v+h)));
  dlat=abs(lat-lat0);
  dh=abs(h-h0);
end
lon=atan2(Y,X)*180/pi;
lat=lat*180/pi;

end
