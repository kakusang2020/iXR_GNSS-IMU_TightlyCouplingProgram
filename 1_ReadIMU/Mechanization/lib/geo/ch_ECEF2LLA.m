function [lat, lon, h] = ch_ECEF2LLA(XYZ)

% ECEF坐标转经纬高
% lat:纬度(rad)
% lon:经度(rad)
% h高度(m)

R_0 = 6378137; %WGS84 Equatorial radius in meters
e = 0.0818191908425; %WGS84 eccentricity


lon = atan2(XYZ(2),XYZ(1));

% From (C.29) and (C.30)
k1 = sqrt(1 - e^2) * abs (XYZ(3));
k2 = e^2 * R_0;
beta = sqrt(XYZ(1)^2 + XYZ(2)^2);
E = (k1 - k2) / beta;
F = (k1 + k2) / beta;

% From (C.31)
P = 4/3 * (E*F + 1);

% From (C.32)
Q = 2 * (E^2 - F^2);

% From (C.33)
D = P^3 + Q^2;

% From (C.34)
V = (sqrt(D) - Q)^(1/3) - (sqrt(D) + Q)^(1/3);

% From (C.35)
G = 0.5 * (sqrt(E^2 + V) + E);

% From (C.36)
T = sqrt(G^2 + (F - V * G) / (2 * G - E)) - G;

% From (C.37)
lat = sign(XYZ(3)) * atan((1 - T^2) / (2 * T * sqrt (1 - e^2)));

% From (C.38)
h = (beta - R_0 * T) * cos(lat) +...
    (XYZ(3) - sign(XYZ(3)) * R_0 * sqrt(1 - e^2)) * sin (lat);

end
