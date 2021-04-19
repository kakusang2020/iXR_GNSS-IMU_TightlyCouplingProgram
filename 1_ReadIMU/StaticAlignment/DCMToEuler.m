function [ Euler ] = DCMToEuler( C )
%DCMTOEULER Summary of this function goes here
%   Detailed explanation goes here
Euler(1,1)=atan2(C(3,2),C(3,3));
Euler(2,1)=atan(-C(3,1)/(sqrt(C(3,2)^2+C(3,3)^2)));
Euler(3,1)=atan2(C(2,1),C(1,1));
end

