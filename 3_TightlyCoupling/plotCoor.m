function plotCoor(Obs_Coor)
lat = [];
lon= [];
h = [];
for i = 1: length(Obs_Coor)
    [lat_ ,lon_ ,h_ ] = XYZ2BLH(Obs_Coor(i,1),Obs_Coor(i,2),Obs_Coor(i,3));
    lat = [lat ; lat_];
    lon = [lon ; lon_];
    h = [h ; h_];
end
geoplot(lat,lon);
%plot(lat,lon);
end

