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
% uif = uifigure;
% g = geoglobe(uif,'Basemap','satellite',"Terrain","gmted2010");
% geoplot3(g,lat,lon,h);
geoplot(lat,lon);
%plot(lat,lon);
end

