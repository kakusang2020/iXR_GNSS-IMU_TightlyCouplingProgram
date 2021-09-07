function [gnssData] = generateGnssData(filePathObs,filePathNav)
%% generate gnss data structure
gnssData = struct('headerData',[],'obsData',[]);
waitBar = waitbar(0,'Generate data','Name','Generate data');
[rinexObs] = loadRINEXO(filePathObs);
[rinexNav] = loadRINEXN(filePathNav);
%% headerData
gnssData.headerData.obsHeader = rinexObs.headerData;
gnssData.headerData.navHeader = rinexNav.headerData;

%% obsData
gnssData.obsData = rinexObs.obsData;
gnssData.navData = rinexNav.navData;
for obsIndex = 1:size(gnssData.obsData,2)
    svObs = gnssData.obsData(obsIndex).svObs;
    obsTime = gnssData.obsData(obsIndex).time;
    for svIndex = 1:size(svObs,2)
        obs = svObs(svIndex);
        constellation = obs.constellation;
        svPRNStr = sprintf([constellation '%03d'],obs.svPRN);
        
        if ~isfield(gnssData.navData.(constellation),svPRNStr) % no such sv in navData
            continue
        end
        
        svNav = gnssData.navData.(constellation).(svPRNStr);
        minTimeDiff = inf;
        for navIndex = 1:size(svNav,2)
            nav = svNav(navIndex);
            timeDiff = (obsTime.GPSWeek - nav.GPSWeek) * 604800 + (obsTime.GPST - nav.toc);
            if strcmp(constellation,'BEIDOU')
                timeDiff = timeDiff - 14;
            end
            if abs(timeDiff) < abs(minTimeDiff)
                minTimeDiff = timeDiff;
                minNavIndex = navIndex;
            end
        end
 
        if ~( abs(minTimeDiff) < 3600 * 2 )
            continue
        end

        gnssData.obsData(obsIndex).svObs(svIndex).navDataTime.navIndex = minNavIndex;
        gnssData.obsData(obsIndex).svObs(svIndex).navDataTime.toc = svNav(minNavIndex).toc;
        gnssData.obsData(obsIndex).svObs(svIndex).navDataTime.GPSWeek = svNav(minNavIndex).GPSWeek;

        % calculate satellite position
        gnssData.obsData(obsIndex).svObs(svIndex).svPosition = calSvPos(obsTime,obs,nav,gnssData.headerData.obsHeader);
    end
    
    waitbar(obsIndex/size(gnssData.obsData,2),waitBar,sprintf('Generate obs...Count:%d ; TOW:%9.3f',obsIndex,obsTime.GPST));
end

close(waitBar);
end