Mpos = zeros(3,length(GNSSTCData))';
for i = 1 : length(GNSSTCData)
   x= GNSSTCData(i,3);
   y= GNSSTCData(i,4);
   z= GNSSTCData(i,5);
   [lat,lon,h]=XYZ2BLH(x,y,z);
   [Mx,My] = lonLat2WebMercator(lat,lon);
   Mpos(i,:) = [GNSSTCData(i,1),Mx,My];
end