%% �������Ǹ�����(el), �ͷ�λ��(az)
function [az, el] = satellite_az_el(sv_pos_ecef, usr_pos_ecef)
% get_satellite_az_el: computes the satellite azimuth and elevation
% given the position of the user and the satellite in ECEF
% Input Args
% : sv_pos_ecef:    ����λ��ECEF
%  usr_pos_ecef:    �û�λ��ECEF
% Output Args: azimuth and elevation
% �ο� GPS ԭ�����ջ���� л��
[lat, lon, ~] = ch_ecef2lla(usr_pos_ecef);

Cecef2enu = [-sin(lon) cos(lon) 0; -sin(lat)*cos(lon) -sin(lat)*sin(lon) cos(lat); cos(lat)*cos(lon) cos(lat)*sin(lon) sin(lat)];

enu = Cecef2enu*[sv_pos_ecef - usr_pos_ecef];

% nroamlized line of silght vector
enu = enu / norm(enu);

az = atan2(enu(1), enu(2));
el = asin(enu(3)/norm(enu));
end


