function [x,y] = lonLat2WebMercator(lat,lon)
x = lon *20037508.34/180;  
y = log(tan((90+lat)*pi/360))/(pi/180);  
y = y *20037508.34/180;   
end

