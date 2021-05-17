function ENUPos = ECEF2ENUPos(out_profile)
% XYZ_ = out_profile(1,2:4);
 XYZ_ = [-3961764.9058;3349009.4333;3698311.9198]';
ENUPos = out_profile;
for i = 1:length(out_profile)
   [lat,lon,h]=XYZ2BLH(out_profile(i,2),out_profile(i,3),out_profile(i,4));
   XYZ = out_profile(i,2:4) - XYZ_;
   ENU = [      -sin(lon)             cos(lon)          0;...
    -sin(lat)*cos(lon)   -sin(lat)*sin(lon)   cos(lat);...
     cos(lat)*cos(lon)    cos(lat)*sin(lon)   sin(lat) ]*XYZ';
 ENUPos(i,2:4) = ENU';
end
