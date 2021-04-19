function [ Q ] = DCMToQuater( C )
%DCMTOQUATER Summary of this function goes here
%   Detailed explanation goes here
Euler=DCMToEuler(C);
Q=EulerToQuater(Euler);
end

