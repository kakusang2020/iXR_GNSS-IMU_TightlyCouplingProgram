function [rinexNav] = loadRINEXN(filePath)
%% generate rinex data structure
rinexNav = struct('headerData',[],'navData',[]);

%% open file
fid = fopen(filePath);

%% read header
headerData = struct('fileVer',[],'ionosphericCorr',[]);
headerData.ionosphericCorr = struct('GPSA',[],'GPSB',[],'BDSA',[],'BDSB',[],'GAL',[],'QZSA',[],'QZSB',[],'IRNA',[],'IRNB',[]);
while true
    line = fgetl(fid);
    if contains(line,'RINEX VERSION / TYPE')
        
        headerData.fileVer = real(str2doubleq(line(1:9)));
        
    elseif contains(line,'IONOSPHERIC CORR')
        ionoCorrType = line(1:4);
        if strcmp(ionoCorrType,'GAL ')
            ionoCorrType = 'GAL';
        end
        ionmsg = nan(1,4);
        for i = 1:4
            ionmsg(1,i) = real(str2doubleq( line( (i-1)*12+6 : i*12+5 ) ));
        end
        
        headerData.ionosphericCorr.(ionoCorrType) = ionmsg; % ionoCorrType 为 动态字段
        
    elseif contains(line,'END OF HEADER')
        break;
    end
end
rinexNav.headerData = headerData;

%% read nav
% only GPS and BEIDOU navgation message are read here
navData = struct('GPS',[],'BEIDOU',[],'GLONASS',[],'Galileo',[],'QZSS',[],'SBAS',[],'IRNSS',[]);
while ~feof(fid)
    line = fgetl(fid);
    constellation = line(1);
    svPRN = real(str2doubleq(line(2:3)));
    switch constellation
        case 'G'
            svPRNStr = num2str(svPRN,'GPS%03d');

            navmsg.year  = real(str2doubleq(line(4:8)));
            navmsg.month = real(str2doubleq(line(9:11)));
            navmsg.day   = real(str2doubleq(line(12:14)));
            navmsg.hour  = real(str2doubleq(line(15:17)));
            navmsg.min= real(str2doubleq(line(18:20)));
            navmsg.sec= real(str2doubleq(line(21:23)));
            [navmsg.toc,navmsg.GPSWeek]=Date2GPSTime(navmsg.year,navmsg.month,navmsg.day,navmsg.hour+navmsg.min/60+navmsg.sec/3600);
            
            navmsg.af0   = real(str2doubleq(line(24:42)));%Satellite clock deviation(s)
            navmsg.af1   = real(str2doubleq(line(43:61)));%Satellite clock drift(s/s)
            navmsg.af2   = real(str2doubleq(line(62:80)));%Drift velocity of satellite clock(s/s*s)
            
            line = fgetl(fid);
            navmsg.IODE  = real(str2doubleq(line(5:23)));
            navmsg.crs   = real(str2doubleq(line(24:42)));%The difference between the average angular velocity calculated by precise ephemeris and the average angular velocity calculated by given parameters
            navmsg.deltan= real(str2doubleq(line(43:61)));
            navmsg.M0    = real(str2doubleq(line(62:80)));%The angle of the reference point

            line = fgetl(fid);
            navmsg.cuc   = real(str2doubleq(line(5:23)));
            navmsg.ecc   = real(str2doubleq(line(24:42)));%Track eccentricity
            navmsg.cus   = real(str2doubleq(line(43:61)));
            navmsg.roota = real(str2doubleq(line(62:80)));%The square root of the long half axis of the orbit

            line = fgetl(fid);
            navmsg.toe   = real(str2doubleq(line(5:23)));%Ephemeris reference time
            navmsg.cic   = real(str2doubleq(line(24:42)));
            navmsg.Omega0= real(str2doubleq(line(43:61)));%Right ascension of ascending node at reference time
            navmsg.cis   = real(str2doubleq(line(62:80)));

            line = fgetl(fid);	
            navmsg.i0       = real(str2doubleq(line(5:23)));%Orbital inclination at reference time
            navmsg.crc      = real(str2doubleq(line(24:42)));
            navmsg.omega    = real(str2doubleq(line(43:61)));%Perigee angle
            navmsg.Omegadot = real(str2doubleq(line(62:80)));

            line = fgetl(fid);	   
            navmsg.idot     = real(str2doubleq(line(5:23)));%Rate of change of track inclination
            navmsg.codesOnL2= real(str2doubleq(line(24:42)));
            navmsg.weekno   = real(str2doubleq(line(43:61)));
            navmsg.L2flag   = real(str2doubleq(line(62:80)));

            line = fgetl(fid); %not be used
            navmsg.svaccur  = real(str2doubleq(line(5:23)));
            navmsg.svhealth = real(str2doubleq(line(24:42)));
            navmsg.tgd1      = real(str2doubleq(line(43:61)));
            navmsg.IODC     = real(str2doubleq(line(62:80)));
            
            line = fgetl(fid); %not be used
            navmsg.tom = real(str2doubleq(line(5:23)));
            navmsg.fit = real(str2doubleq(line(24:42)));

            if ~isfield(navData.GPS,svPRNStr)
                navIndex = 1;
                navData.GPS.(svPRNStr)(navIndex) = navmsg;
            else
                navIndex = size(navData.GPS.(svPRNStr),2) + 1;
                navData.GPS.(svPRNStr)(navIndex) = navmsg;
            end
        case 'C'
%             svPRNStr = num2str(svPRN,'BEIDOU%03d');
%             
%             navmsg.year  = real(str2doubleq(line(4:8)));
%             navmsg.month = real(str2doubleq(line(9:11)));
%             navmsg.day   = real(str2doubleq(line(12:14)));
%             navmsg.hour  = real(str2doubleq(line(15:17)));
%             navmsg.min= real(str2doubleq(line(18:20)));
%             navmsg.sec= real(str2doubleq(line(21:23)));
%             [navmsg.toc,navmsg.GPSWeek]=Date2GPSTime(navmsg.year,navmsg.month,navmsg.day,navmsg.hour+navmsg.min/60+navmsg.sec/3600);
%             
%             navmsg.af0   = real(str2doubleq(line(24:42)));
%             navmsg.af1   = real(str2doubleq(line(43:61)));
%             navmsg.af2   = real(str2doubleq(line(62:80)));
%             
%             line = fgetl(fid);	 
%             navmsg.IODE  = real(str2doubleq(line(5:23)));
%             navmsg.crs   = real(str2doubleq(line(24:42)));
%             navmsg.deltan= real(str2doubleq(line(43:61)));
%             navmsg.M0    = real(str2doubleq(line(62:80)));
% 
%             line = fgetl(fid);
%             navmsg.cuc   = real(str2doubleq(line(5:23)));
%             navmsg.ecc   = real(str2doubleq(line(24:42)));
%             navmsg.cus   = real(str2doubleq(line(43:61)));
%             navmsg.roota = real(str2doubleq(line(62:80)));
% 
%             line = fgetl(fid);
%             navmsg.toe   = real(str2doubleq(line(5:23)));
%             navmsg.cic   = real(str2doubleq(line(24:42)));
%             navmsg.Omega0= real(str2doubleq(line(43:61)));
%             navmsg.cis   = real(str2doubleq(line(62:80)));
% 
%             line = fgetl(fid);	
%             navmsg.i0       = real(str2doubleq(line(5:23)));
%             navmsg.crc      = real(str2doubleq(line(24:42)));
%             navmsg.omega    = real(str2doubleq(line(43:61)));
%             navmsg.Omegadot = real(str2doubleq(line(62:80)));
% 
%             line = fgetl(fid);	   
%             navmsg.idot     = real(str2doubleq(line(5:23)));
%             
%             navmsg.weekno   = real(str2doubleq(line(43:61)));
%             
% 
%             line = fgetl(fid);
%             navmsg.svaccur  = real(str2doubleq(line(5:23)));
%             navmsg.svhealth = real(str2doubleq(line(24:42)));
%             navmsg.tgd1     = real(str2doubleq(line(43:61)));
%             navmsg.tgd2     = real(str2doubleq(line(62:80)));
% 
%             line = fgetl(fid);
%             navmsg.tom = real(str2doubleq(line(5:23)));
%             navmsg.IODC = real(str2doubleq(line(24:42)));
%             
%             if ~isfield(navData.BEIDOU,svPRNStr)
%                 navIndex = 1;
%                 navData.BEIDOU.(svPRNStr)(navIndex) = navmsg;
%             else
%                 navIndex = size(navData.BEIDOU.(svPRNStr),2) + 1;
%                 navData.BEIDOU.(svPRNStr)(navIndex) = navmsg;
%             end
        case 'R'
            line = fgetl(fid);
            line = fgetl(fid);
            line = fgetl(fid);
        case 'E'
            line = fgetl(fid);
            line = fgetl(fid);
            line = fgetl(fid);
            line = fgetl(fid);
            line = fgetl(fid);
            line = fgetl(fid);
            line = fgetl(fid);
        case 'J'
            svPRN = svPRN + 32;
            svPRNStr = num2str(svPRN,'QZSS%03d');
            navmsg.year  = real(str2doubleq(line(4:8)));
            navmsg.month = real(str2doubleq(line(9:11)));
            navmsg.day   = real(str2doubleq(line(12:14)));
            navmsg.hour  = real(str2doubleq(line(15:17)));
            navmsg.min= real(str2doubleq(line(18:20)));
            navmsg.sec= real(str2doubleq(line(21:23)));
            [navmsg.toc,navmsg.GPSWeek]=Date2GPSTime(navmsg.year,navmsg.month,navmsg.day,navmsg.hour+navmsg.min/60+navmsg.sec/3600);
            
            navmsg.af0   = real(str2doubleq(line(24:42)));%Satellite clock deviation（s）
            navmsg.af1   = real(str2doubleq(line(43:61)));%Satellite clock drift（s/s）
            navmsg.af2   = real(str2doubleq(line(62:80)));%Drift velocity of satellite clock（s/s*s）
            
            line = fgetl(fid);
            navmsg.IODE  = real(str2doubleq(line(5:23)));
            navmsg.crs   = real(str2doubleq(line(24:42)));%The difference between the average angular velocity calculated by precise ephemeris and the average angular velocity calculated by given parameters
            navmsg.deltan= real(str2doubleq(line(43:61)));
            navmsg.M0    = real(str2doubleq(line(62:80)));%The angle of the reference point

            line = fgetl(fid);
            navmsg.cuc   = real(str2doubleq(line(5:23)));
            navmsg.ecc   = real(str2doubleq(line(24:42)));%Track eccentricity
            navmsg.cus   = real(str2doubleq(line(43:61)));
            navmsg.roota = real(str2doubleq(line(62:80)));%The square root of the long half axis of the orbit

            line = fgetl(fid);
            navmsg.toe   = real(str2doubleq(line(5:23)));%Ephemeris reference time
            navmsg.cic   = real(str2doubleq(line(24:42)));
            navmsg.Omega0= real(str2doubleq(line(43:61)));%Right ascension of ascending node at reference time
            navmsg.cis   = real(str2doubleq(line(62:80)));

            line = fgetl(fid);	
            navmsg.i0       = real(str2doubleq(line(5:23)));%Orbital inclination at reference time
            navmsg.crc      = real(str2doubleq(line(24:42)));
            navmsg.omega    = real(str2doubleq(line(43:61)));%Perigee angle
            navmsg.Omegadot = real(str2doubleq(line(62:80)));

            line = fgetl(fid);	   
            navmsg.idot     = real(str2doubleq(line(5:23)));%Rate of change of track inclination
            navmsg.codesOnL2= real(str2doubleq(line(24:42)));
            navmsg.weekno   = real(str2doubleq(line(43:61)));
            navmsg.L2flag   = real(str2doubleq(line(62:80)));

            line = fgetl(fid); %not be used
            navmsg.svaccur  = real(str2doubleq(line(5:23)));
            navmsg.svhealth = real(str2doubleq(line(24:42)));
            navmsg.tgd1      = real(str2doubleq(line(43:61)));
            navmsg.IODC     = real(str2doubleq(line(62:80)));
            
            line = fgetl(fid); %not be used
            navmsg.tom = real(str2doubleq(line(5:23)));
            navmsg.fit = real(str2doubleq(line(24:42)));

            if ~isfield(navData.QZSS,svPRNStr)
                navIndex = 1;
                navData.QZSS.(svPRNStr)(navIndex) = navmsg;
            else
                navIndex = size(navData.QZSS.(svPRNStr),2) + 1;
                navData.QZSS.(svPRNStr)(navIndex) = navmsg;
            end
        case 'S'
            line = fgetl(fid);
            line = fgetl(fid);
            line = fgetl(fid);
        case 'I'
            line = fgetl(fid);
            line = fgetl(fid);
            line = fgetl(fid);
            line = fgetl(fid);
            line = fgetl(fid);
            line = fgetl(fid);
            line = fgetl(fid);
        otherwise
            error 'Unrecognized constellation'
    end
end

rinexNav.navData = navData;

end