% subplot���Ƿ���subplot
function ch_plot_imu(varargin)
%%  plot imu data
i = 1;
param= inputParser;
param.addOptional('time', []);
param.addOptional('acc', []);
param.addOptional('gyr', []);
param.addOptional('mag', []);
param.addOptional('eul', []);
param.addOptional('gb', []); % ���ٶ���ƫ
param.addOptional('wb', []); % ������ƫ
param.addOptional('phi', []); %ʧ׼��
param.addOptional('P_phi', []); %ʧ׼�Ƿ���
param.addOptional('P_wb', []); %���ݷ���
param.addOptional('P_pos', []); %λ�÷���
param.addOptional('title', []);
param.addOptional('legend', []);


%Ȼ������Ĳ������д�������в�ͬ��Ĭ��ֵ���Ǿ͸��ǵ�
param.parse(varargin{:});
r = param.Results;

if(r.time == 0 )
    error('no time data');
end
i = 1;

figure;

if(~isempty(r.gyr))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.gyr, {'X', 'Y', 'Z'}, 'Time (s)', 'Angular rate (dps(deg /s))', 'Gyroscope');
end

if(~isempty(r.acc))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.acc, {'X', 'Y', 'Z'}, 'Time (s)', 'Acceleration (g)', 'Accelerometer');
end

if(~isempty(r.mag))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.mag, {'X', 'Y', 'Z'}, 'Time (s)', 'Flux (G)', 'Magnetometer');
end

if(~isempty(r.eul))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.eul, {'X', 'Y', 'Z'}, 'Time (s)', 'Angle(deg)', 'Eular Angle');
end

if(~isempty(r.wb))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.wb, {'X', 'Y', 'Z'}, 'Time (s)', 'Angle(deg)', '������ƫ');
end

if(~isempty(r.gb))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.gb, {'X', 'Y', 'Z'}, 'Time (s)', 'm/s^(2)', '���ٶ���ƫ');
end

if(~isempty(r.phi))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.phi, {'X', 'Y', 'Z'}, 'Time (s)', 'Angle(deg)', 'ʧ׼��');
end

if(~isempty(r.P_phi))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.P_phi, {'X', 'Y', 'Z'}, 'Time (s)', '-', 'Phi Var(ʧ׼�Ƿ���)');
end


if(~isempty(r.P_wb))
    subplot(2,2,i);
    i = i+1;
    interial_display(r.time,  r.P_wb, {'X', 'Y', 'Z'}, 'Time (s)', '-', '������ƫ����');
end


if(~isempty(r.P_pos))
    subplot(2,2,i);
    interial_display(r.time,  r.P_pos, {'X', 'Y', 'Z'}, 'Time (s)', '-', 'λ�÷���');
end

%    linkaxes(axis, 'x');

end


function interial_display(time, data, legendstr, xlabelstr, ylabelstr, titlestr)
hold on;
plot(time, data(:,1), 'r');
plot(time, data(:,2), 'g');
plot(time, data(:,3), 'b');
legend(legendstr);
xlabel(xlabelstr);
ylabel(ylabelstr);
title(titlestr);
hold off;
end
