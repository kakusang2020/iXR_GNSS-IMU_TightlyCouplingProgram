function a=CalIncludeAngle(x,y)
% 求两条直线夹角
% x,y 是已知三点的横坐标和纵坐标
% eg: x=[1 2 3];y=[4 1 5];
if x(2) ~= x(1)
    k1=(y(2)-y(1))/(x(2)-x(1));
end
if x(3)~=x(2)
    k2=(y(3)-y(2))/(x(3)-x(2));
end
if x(2)==x(1) && x(3)==x(2)
    a=0;
elseif x(3)==x(2)
    a=pi/2-atan(abs(k1));
elseif x(1)==x(2)
    a=pi/2-atan(abs(k2));
elseif 1+k1*k2==0
    a=pi/2;
else
    a=atan(abs((k2-k1)/(1+k2*k1))); % 夹角
end
a=a*360/(2*pi) - 90; % 转化为角度制
end