function [rinexObs] = loadRINEXO(filePath)%% generate rinex data structure
rinexObs = struct('headerData',[],'obsData',[]);

%% open file
waitBar = waitbar(0,'Open RINEX observation file...','Name','Reading RINEX observation');
fid = fopen(filePath);

%% read header
waitbar(0,waitBar,'Read header...');
headerData = struct('fileVer',[],'approPos',[],'interval',[],'obsType',[]);
headerData.interval = 1; % default interval
headerData.obsType = struct('GPS',[],'BEIDOU',[],'GLONASS',[],'Galileo',[],'QZSS',[],'SBAS',[]);
while true
    line = fgetl(fid);
    if contains(line,'RINEX VERSION / TYPE')
        
        headerData.fileVer = real(str2doubleq(line(1:9)));
        
    elseif contains(line,'APPROX POSITION XYZ')
        wgs84 = wgs84Ellipsoid('meters');
        approxPosECEF = sscanf(line,'%f' ,3);
        [approxPosGeo(1),approxPosGeo(2),approxPosGeo(3)] = ecef2geodetic(approxPosECEF(1),approxPosECEF(2),approxPosECEF(3),wgs84);
        approPos.approxPosECEF = approxPosECEF;
        approPos.approxPosGeo = approxPosGeo;
        
        headerData.approPos = approPos;
        
    elseif contains(line,'INTERVAL')
        
        headerData.interval = real(str2doubleq(line(5:10)));
        
    elseif contains(line,'TIME OF FIRST OBS')
        
        headerData.First_Obs=[str2double(line(5:6)) str2double(line(11:12)) str2double(line(17:18))];
        headerData.DOY=UTC2MJD(str2double(line(5:6)),str2double(line(11:12)),str2double(line(17:18)),0,0,0) ...
                    -UTC2MJD(str2double(line(5:6)),1,1,0,0,0)+1;   
        
    elseif contains(line,'SYS / # / OBS TYPES')
        constellation = line(1);
        nObservables = real(str2doubleq(line(4:6)));
        splitLine = strsplit(line(8:59));
        observables = splitLine(1:end-1);
        while nObservables > 13
            line = fgetl(fid);
            splitLine = strsplit(line(8:59));
            observables = [observables splitLine(1:end-1)];
            nObservables = nObservables - 13;
        end
        
        switch constellation
            case 'G'
                headerData.obsType.GPS = observables;
                for obsTypeIndex = 1:size(observables,2)
                    headerData.obsType.typeIndexGPS.(observables{obsTypeIndex}) = obsTypeIndex;
                end
            case 'C'
                headerData.obsType.BEIDOU = observables;
                for obsTypeIndex = 1:size(observables,2)
                    headerData.obsType.typeIndexBEIDOU.(observables{obsTypeIndex}) = obsTypeIndex;
                end
            case 'R'
                headerData.obsType.GLONASS = observables;
                for obsTypeIndex = 1:size(observables,2)
                    headerData.obsType.typeIndexGLONASS.(observables{obsTypeIndex}) = obsTypeIndex;
                end
            case 'E'
                headerData.obsType.Galileo = observables;
                for obsTypeIndex = 1:size(observables,2)
                    headerData.obsType.typeIndexGalileo.(observables{obsTypeIndex}) = obsTypeIndex;
                end
            case 'J'
                headerData.obsType.QZSS = observables;
                for obsTypeIndex = 1:size(observables,2)
                    headerData.obsType.typeIndexQZSS.(observables{obsTypeIndex}) = obsTypeIndex;
                end
            case 'S'
                headerData.obsType.SBAS = observables;
                for obsTypeIndex = 1:size(observables,2)
                    headerData.obsType.typeIndexSBAS.(observables{obsTypeIndex}) = obsTypeIndex;
                end
            otherwise
                error 'Unrecognized constellation'
        end
        
    elseif contains(line,'END OF HEADER')
        break;
    end
end
rinexObs.headerData = headerData;

%% read obs
obsCount = 0;
waitbar(0,waitBar,'Read obs...');
obsData = struct('time',[],'obsInfo',[],'svObs',[],'svPRNIndex',[]);
while ~feof(fid)
    line = fgetl(fid);
    time = struct('GPSWeek',[],'GPST',[],'year',[],'month',[],'day',[],'hour',[],'min',[],'sec',[]);
    obsInfo = struct('epochFlag',[],'nSat',[]);
    if line(1) == '>'
        time.year = real(str2doubleq(line(3:6)));
        time.month = real(str2doubleq(line(8:9)));
        time.day = real(str2doubleq(line(11:12)));
        time.hour = real(str2doubleq(line(14:15)));
        time.min = real(str2doubleq(line(17:18)));
        time.sec = real(str2doubleq(line(19:29)));
        [time.GPST,time.GPSWeek]=Date2GPSTime(time.year,time.month,time.day,time.hour+time.min/60+time.sec/3600);
        
        obsInfo.epochFlag = real(str2doubleq(line(32)));
        obsInfo.nSat = real(str2doubleq(line(33:35)));
        
        obsCount = obsCount + 1;
        obsData(obsCount).time = time;
        obsData(obsCount).obsInfo = obsInfo;

        waitbar(mod(obsCount,10000)/10000,waitBar,sprintf('Read obs...Count:%d ; TOW:%9.3f',obsCount,time.GPST)); % the bar is fake but count is true
        
    else
        disp(' ''>'' missing')
        continue
    end

    svObs = repmat(struct('constellation',[],'svPRN',[],'measurements',[]),[1,obsInfo.nSat]);
    for svIndex = 1:obsInfo.nSat
        line = fgetl(fid);
%         svObs(svIndex).svPRN = real(str2doubleq(line(2:3)));
        constellation = line(1);
        switch constellation
            case 'G'
                svObs(svIndex).constellation = 'GPS';
                nMeasurements = size(headerData.obsType.GPS,2);
                svObs(svIndex).svPRN = real(str2doubleq(line(2:3)));
            case 'C'
                svObs(svIndex).constellation = 'BEIDOU';
                nMeasurements = size(headerData.obsType.BEIDOU,2);
            case 'R'
                svObs(svIndex).constellation = 'GLONASS';
                nMeasurements = size(headerData.obsType.GLONASS,2);
            case 'E'
                svObs(svIndex).constellation = 'Galileo';
                nMeasurements = size(headerData.obsType.Galileo,2);
            case 'J'
                svObs(svIndex).constellation = 'QZSS';
                nMeasurements = size(headerData.obsType.QZSS,2);
                svObs(svIndex).svPRN = real(str2doubleq(line(2:3))) + 32;
            case 'S'
                svObs(svIndex).constellation = 'SBAS';
                nMeasurements = size(headerData.obsType.SBAS,2);
            otherwise
                error 'Unrecognized constellation'
        end
        
        obsData(obsCount).svPRNIndex.(svObs(svIndex).constellation)(svObs(svIndex).svPRN) = svIndex;
        svObs(svIndex).measurements = nan(1,nMeasurements);
        for measurementIndex = 1:nMeasurements
            if 16*measurementIndex+1 <= length(line)
                measure = real(str2doubleq(...
                    line(16*(measurementIndex-1)+4:16*measurementIndex+1) ...
                ));
                if measure ~= 0
                    svObs(svIndex).measurements(measurementIndex) = measure;
                end
            else
                continue;
            end
        end
    end
    obsData(obsCount).svObs = svObs;
end
rinexObs.obsData = obsData;

close(waitBar);
end