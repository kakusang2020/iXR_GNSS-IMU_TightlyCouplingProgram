function [az] = CalHeading(ECEFPos1,ECEFPos2)
% Algorithm Studio
% Author : CloudWalker
% Date: 2020-08-10
% Email:jordan2333@aliyun.com
% Function: Example1-根据p1和p2经纬度估算两点的距离和方位角(p1 → p2)。
%           Example2-根据p1以及航向和距离，计算终点p2的位置以及方位角。
% Reference: http://www.movable-type.co.uk/scripts/latlong-vincenty.html

% digitsOld = digits(10);
%% Example 1
UbloxData = csvread('Ublox0412.csv');
for i = 2:size(UbloxData,1)-250
    ECEFPos1 = UbloxData(i-1,2:4);
    ECEFPos2 = UbloxData(i+250,2:4);
[lat1,lon1,h1] = XYZ2BLH(ECEFPos1(1),ECEFPos1(2),ECEFPos1(3));
[lat2,lon2,h2] = XYZ2BLH(ECEFPos2(1),ECEFPos2(2),ECEFPos2(3));
az = azimuth(lat1,lon1,lat2,lon2);
UbloxData(i,5) = az;
end
% lon1 = deg2rad(104.628601) ; lat1 = deg2rad(29.380394);  %p1	
% lon2 = deg2rad(104.628602); lat2 = deg2rad(29.380394);   %p2
% f = 1 /  298.257223563;
% a= 6378137.0;	
% b= 6356752.314245;
% 
% L = lon2 - lon1;
% tanU1 = (1-f)*tan(lat1); cosU1 = 1 / sqrt((1 + tanU1*tanU1));sinU1 = tanU1 * cosU1;
% tanU2 = (1-f)*tan(lat2); cosU2 = 1 / sqrt((1 + tanU2*tanU2));sinU2 = tanU2 * cosU2;
% lambda = L;
% lambda_ = 0;
% iterationLimit = 100;
% while (abs(lambda - lambda_) > 1e-12 && iterationLimit>0)
%         iterationLimit = iterationLimit -1;
%         sinlambda = sin(lambda);
%         coslambda = cos(lambda);
%         sinSq_delta = (cosU2*sinlambda) * (cosU2*sinlambda) + (cosU1*sinU2-sinU1*cosU2* coslambda) * (cosU1*sinU2-sinU1*cosU2* coslambda);
%         sin_delta = sqrt(sinSq_delta);
%         if sin_delta==0 
%                return 
%         end
%         cos_delta = sinU1*sinU2 + cosU1*cosU2*coslambda;
%         delta = atan2(sin_delta, cos_delta);
%         sin_alpha = cosU1 * cosU2 * sinlambda / sin_delta;
%         cosSq_alpha = 1 - sin_alpha*sin_alpha;
%         cos2_deltaM = cos_delta - 2*sinU1*sinU2/cosSq_alpha;
%         C = f/16*cosSq_alpha*(4+f*(4-3*cosSq_alpha));
%         lambda_ = lambda;
%         lambda = L + (1-C) * f * sin_alpha * (delta + C*sin_delta*(cos2_deltaM+C*cos_delta*(-1+2*cos2_deltaM*cos2_deltaM)));
% end
% uSq = cosSq_alpha * (a*a - b*b) / (b*b);
% A = 1 + uSq/16384*(4096+uSq*(-768+uSq*(320-175*uSq)));
% B = uSq/1024 * (256+uSq*(-128+uSq*(74-47*uSq)));
% delta_delta = B*sin_delta*(cos2_deltaM+B/4*(cos_delta*(-1+2*cos2_deltaM*cos2_deltaM)-B/6*cos2_deltaM*(-3+4*sin_delta*sin_delta)*(-3+4*cos2_deltaM*cos2_deltaM)));
% s = b*A*(delta-delta_delta);
% fwdAz = atan2(cosU2*sinlambda,  cosU1*sinU2-sinU1*cosU2*coslambda); %初始方位角
% revAz = atan2(cosU1*sinlambda, -sinU1*cosU2+cosU1*sinU2*coslambda); %最终方位角

%% Example2
% lon1 = deg2rad(104.628601) ; lat1 = deg2rad(29.380394);  %p1	
% alpha_1 = deg2rad(fwdAz * 180 / pi + 360); %  方向角
% s = s; %距离
% 
% sin_alpha1 = sin(alpha_1);
% cos_alpha1 = cos(alpha_1);
% 
% tanU1 = (1-f) * tan(lat1); cosU1 = 1 / sqrt((1 + tanU1*tanU1)); sinU1 = tanU1 * cosU1;
% delta_1 = atan2(tanU1, cos_alpha1);
% sin_alpha = cosU1 * sin_alpha1;
% cosSq_alpha = 1 - sin_alpha*sin_alpha;
% uSq = cosSq_alpha * (a*a - b*b) / (b*b);
% A = 1 + uSq/16384*(4096+uSq*(-768+uSq*(320-175*uSq)));
% B = uSq/1024 * (256+uSq*(-128+uSq*(74-47*uSq)));
% 
% delta = s / (b*A);
% dleta_ = 0;
% while abs(delta-dleta_) > 1e-12
%     cos2deltaM = cos(2*delta_1 + delta);
%     sin_delta = sin(delta);
%     cos_delta = cos(delta);
%     delta_delta = B*sin_delta*(cos2deltaM+B/4*(cos_delta*(-1+2*cos2deltaM*cos2deltaM)-B/6*cos2deltaM*(-3+4*sin_delta*sin_delta)*(-3+4*cos2deltaM*cos2deltaM)));
%     dleta_ = delta;
%     delta = s / (b*A) + delta_delta;
% end
% 
% tmp = sinU1*sin_delta - cosU1*cos_delta*cos_alpha1;
% lat2 = atan2(sinU1*cos_delta + cosU1*sin_delta*cos_alpha1, (1-f)*sqrt(sin_alpha*sin_alpha + tmp*tmp)); %目标点纬度
% lon =  atan2(sin_delta*sin_alpha1, cosU1*cos_delta - sinU1*sin_delta*cos_alpha1);
% C = f/16*cosSq_alpha*(4+f*(4-3*cosSq_alpha));
% L = lon - (1-C) * f * sin_alpha *(delta + C*sin_delta*(cos2deltaM+C*cos_delta*(-1+2*cos2deltaM*cos2deltaM)));
% lon2 =roundn(rem((lon1+L+3*pi) ,(2*pi)) - pi, -6);  %normalise to -180...+180 目标点精度
% 
% revAz = atan2(sin_alpha, -tmp); %最终方位角
% ————————————————
% 版权声明：本文为CSDN博主「AL.CK」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
% 原文链接：https://blog.csdn.net/linkcian/article/details/107921540

