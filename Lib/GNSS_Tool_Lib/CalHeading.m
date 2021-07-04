function [az] = CalHeading(ECEFPos1,ECEFPos2)

[lat1,lon1,h1] = XYZ2BLH(ECEFPos1(1),ECEFPos1(2),ECEFPos1(3));
[lat2,lon2,h2] = XYZ2BLH(ECEFPos2(1),ECEFPos2(2),ECEFPos2(3));
az = azimuth(lat1,lon1,lat2,lon2);
end


