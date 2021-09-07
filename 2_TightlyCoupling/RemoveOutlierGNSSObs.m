function [ index_elimi,RecClockPre,RecClockBias,Innovation,RecClockRateBias] = RemoveOutlierGNSSObs( RecPosi,RecVelo,GNSSObs,est_clock_previous,tor_s,RecClockPreprocOptions,GNSS_config )
%Remove the observation whose elevation angle and carrier to noise ratio do not meet the threshold requirements
%Before performing the positioning calculation, the clock difference of the receiver is calculated by the prior receiver position, and each pseudo range observation can get a clock difference
%The median of each satellite clock error is used as the estimated receiver clock error recclockpre
%If the difference between the clock difference calculated by a satellite and recclockpre (that is, the innovation calculated by using recclockpre as the clock difference) is greater than recclockbias threshold, it is rejected
%Recclockbias is equal to innovation when recclockprecoptions = 2. When recclockprecoptions = 0, the clock difference of receiver used by innovation is calculated by one step recursive method
% Inputs:
% RecPosi           A priori GNSS receiver antenna location
% RecVelo           A priori GNSS receiver antenna velocity
% GNSSObs        GNSS observation array after preprocessing
%  Column 1: epoch
%  Column 2: Obsweek (week)
%  Column 3: Obssec (s)
%  Column 4: PRN
%  Column 5: corrected pseudorange (m)
%  Column 6-8: Satellite position in ECEF(m)
%  Column 9: range rate(m/s)
%  Column 10: Rate of Satellite clock  (s/s)
%  Column 11-13: Satellite velocity in ECEF(m/s)
%  Column 14: Elevation angle (deg)
%  Column 15: SNR
% est_clock_previous         Receiver clock differential clock speed obtained by last Kalman filtering
%  Column 1: Ranging error caused by receiver clock error (m)
%  Column 2: Receiver clock speed (m/s)
% tor_s  Time interval£¨s£©
% RecClockPreprocOptions        0 means to use the traditional model in the whole process, 2 means to use the clock differential repair preprocessing strategy in the whole process
% GNSS_config
%     .epoch_interval     Interval between GNSS epochs (s)
%     .init_est_r_ea_e    Initial estimated position (m; ECEF)
%     .mask_angle         Mask angle (deg)
%     .mask_SignalStrenth Mask Carrier to noise ratio(dbhz)
%     .rx_clock_offset    Receiver clock offset at time=0 (m)
%     .rx_clock_drift     Receiver clock drift at time=0 (m/s)
%     .intend_no_GNSS_meas       Max number of satellites
% Outputs:
% RecClockPre: Ranging error caused by prior receiver clock error (m)
% index_elimi: 0 for reservation, 1 for elimination of satellites, and satellite sequence corresponds to gnssobs sequence
% RecClockBias The difference between the receiver clock difference calculated by each satellite and recclockpre
% RecClockRateBias The difference between the receiver clock speed calculated by each satellite and recclockpre
% Innovation innovation of pseudo range and pseudo range rate
c=299792458.0;   % velocity of light (m/s)
RecClockBiasThreshold=30;
InnovationThreshold=30;
[no_GNSS_meas,~]=size(GNSSObs);
% Calculate the estimated pseudo range
RecClockPre=median(GNSSObs(:,5)-...
     sqrt(dot(GNSSObs(:,6:8)'-RecPosi*ones(1,no_GNSS_meas),GNSSObs(:,6:8)'-RecPosi*ones(1,no_GNSS_meas)))');
%RecClockPre=median(GNSSObs(:,5));
%  Calculate the estimated pseudo range rate and ignore Sagnac
u_as_e_T = zeros(no_GNSS_meas,3);
RecClockRateAll =zeros(no_GNSS_meas,1);
for i=1:no_GNSS_meas
    delta_r =  GNSSObs(i,6:8)' - RecPosi; 
    range = sqrt(delta_r' * delta_r);
    u_as_e_T(i,1:3) = delta_r' / range;
    RecClockRateAll(i,1)= GNSSObs(i,9)  -u_as_e_T(i,1:3) * (GNSSObs(i,11:13)'- RecVelo);%- GNSSObs(i,16)
end
RecClockRatePre=median(RecClockRateAll); 
RecClockBias=GNSSObs(:,5)-...
     sqrt(dot(GNSSObs(:,6:8)'-RecPosi*ones(1,no_GNSS_meas),GNSSObs(:,6:8)'-RecPosi*ones(1,no_GNSS_meas)))'-RecClockPre;
RecClockRateBias= RecClockRateAll-RecClockRatePre;
if RecClockPreprocOptions==0
    RecClockUpdate=est_clock_previous(1)+est_clock_previous(2)*tor_s;
    Innovation=[GNSSObs(:,5)-...
        sqrt(dot(GNSSObs(:,6:8)'-RecPosi*ones(1,no_GNSS_meas),GNSSObs(:,6:8)'-RecPosi*ones(1,no_GNSS_meas)))'-RecClockUpdate*ones(no_GNSS_meas,1);...
        RecClockRateAll-est_clock_previous(2)*ones(no_GNSS_meas,1)];
    index_elimi=abs (Innovation(1:no_GNSS_meas))>InnovationThreshold;
else
    Innovation=[RecClockBias;RecClockRateBias];
    index_elimi=abs (RecClockBias)>RecClockBiasThreshold;
end
index_lowele=GNSSObs(:,14)<GNSS_config.mask_angle;
index_lowSignalStrenth=GNSSObs(:,15)<GNSS_config.mask_SignalStrenth;
index_elimi=index_lowele|index_lowSignalStrenth|index_elimi;
end

