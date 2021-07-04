function ENU = ECEF2ENUPos(out_profile)
% XYZ_ = out_profile(1,2:4);
%  XYZ_ = [-3961904.939,3348993.763,3698211.764];
STA.STA(1).Coor(1:3) = [-3961904.939,3348993.763,3698211.764];
ENU = out_profile;
for i = 1:length(out_profile)
   [lat,lon,h]=XYZ2BLH(out_profile(i,2),out_profile(i,3),out_profile(i,4));
   lat = lat*pi/180;lon=lon*pi/180;
%    XYZ = out_profile(i,2:4) - XYZ_;
%    ENU = [      -sin(lon)             cos(lon)          0;...
%     -sin(lat)*cos(lon)   -sin(lat)*sin(lon)   cos(lat);...
%      cos(lat)*cos(lon)    cos(lat)*sin(lon)   sin(lat) ]*XYZ';
%  ENUPos(i,2:4) = ENU';
%     Rotation=[   -sin(est_L_b)*cos(est_lambda_b),-sin(est_L_b)*sin(est_lambda_b),cos(est_L_b);
%         -sin(est_lambda_b),cos(est_lambda_b),0;
%         cos(est_L_b)*cos(est_lambda_b),cos(est_L_b)*sin(est_lambda_b),sin(est_L_b)];
    Rotation=[   -sin(lat)*cos(lon),-sin(lat)*sin(lon),cos(lat);
        -sin(lon),cos(lon),0;
        cos(lat)*cos(lon),cos(lat)*sin(lon),sin(lat)];
    NEU=Rotation*(out_profile(i,2:4)-STA.STA(1).Coor(1:3))';
    ENU(i,2:4)=[NEU(2),NEU(1),NEU(3)];
end
