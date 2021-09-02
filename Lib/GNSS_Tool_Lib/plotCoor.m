% function plotCoor(Obs_Coor,name)
function plotCoor(Obs_Coor)
lat = [];
lon= [];
h = [];
for i = 1: size(Obs_Coor,1)
    [lat_ ,lon_ ,h_ ] = XYZ2BLH(Obs_Coor(i,1),Obs_Coor(i,2),Obs_Coor(i,3));
    lat = [lat ; lat_];
    lon = [lon ; lon_];
    h = [h ; h_];
end
% name=num2str(name,'%04d');
% geoplot(lat,lon,'bs','MarkerSize',12, 'MarkerFaceColor','b');
geoplot(lat,lon);
% text(lat,lon+0.035,name,'FontSize',16,'FontWeight','Bold');
% geoplot(lat,lon);
%plot(lat,lon);
end

