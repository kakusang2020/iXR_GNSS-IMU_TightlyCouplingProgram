function MJD=UTC2MJD(y,mon,d,h,m,s)

if y<80
    y=y+2000;
else
    y=y+1900;
end

if mon<=2
    y=y-1;mon=mon+12;
end
MJD=fix(365.25*y)+fix(30.6001*(mon+1))+d+(h+(m+s/60)/60)/24+1720981.5-2400000.5;

end

