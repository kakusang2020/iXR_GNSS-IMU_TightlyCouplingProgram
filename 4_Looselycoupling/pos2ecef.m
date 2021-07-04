function [r] = pos2ecef(pos)
% /* transform geodetic to ecef position -----------------------------------------
% * transform geodetic position to ecef position
% * args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
% *          double *r        O   ecef position {x,y,z} (m)
% * return : none
% * notes  : WGS84, ellipsoidal height
% *-----------------------------------------------------------------------------*/
r = zeros(1,3);
FE_WGS84  =  (1.0/298.257223563);
RE_WGS84  =  6378137.0; 
sinp=sin(pos(1));
cosp=cos(pos(1));
sinl=sin(pos(2));
cosl=cos(pos(2));
e2=FE_WGS84*(2.0-FE_WGS84);
v=RE_WGS84/sqrt(1.0-e2*sinp*sinp);

r(1)=(v+pos(3))*cosp*cosl;
r(2)=(v+pos(3))*cosp*sinl;
r(3)=(v*(1.0-e2)+pos(3))*sinp;
end