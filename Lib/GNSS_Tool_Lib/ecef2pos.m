function [pos]=ecef2pos(r)

%X, Y, Z to B, L, H.
%B, L, H degree, H is meter.
% Written by: Zhang Yize
FE_WGS84  =  (1.0/298.257223563);
RE_WGS84  =  6378137.0; 
X = r(1);
Y = r(2);
Z = r(3);
a=RE_WGS84;
e2=FE_WGS84*(2.0-FE_WGS84); %WGS84
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
lon=atan2(Y,X);
pos = [lat,lon,h];
end
