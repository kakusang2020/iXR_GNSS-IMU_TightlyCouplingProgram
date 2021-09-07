function [tow,gpsWeek]=Date2GPSTime(year,month,day,HOD)

number_of_leapyear = 0;
for y = 1980:year-1
    if(mod(y,4) == 0 && mod(y,100) ~= 0)
        number_of_leapyear = number_of_leapyear + 1;
    elseif(mod(y,400) == 0)
        number_of_leapyear = number_of_leapyear + 1;
    end
end

total_day = 0;
total_day = total_day + 365 * (year - 1980) + number_of_leapyear;
total_day = total_day - 5;
days_of_month = [31,28,31,30,31,30,31,31,30,31,30,31];
for m = 1:month-1
    total_day = total_day + days_of_month(1,m);
end
if( month>2 && ((mod(year,4) == 0 && mod(year,100) ~= 0) || (mod(year,400) == 0)) )
    total_day = total_day + 1;
end
total_day = total_day + day - 1;
tow = mod(total_day,7) * 3600 * 24 + HOD * 3600;
gpsWeek = (total_day - mod(total_day,7)) / 7;

end