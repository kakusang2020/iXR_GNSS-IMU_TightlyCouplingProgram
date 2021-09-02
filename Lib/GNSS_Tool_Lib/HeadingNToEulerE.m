function Euler_E = HeadingNToEulerE( heading_n,est_L_b,est_lambda )
%������ˮƽ����ϵ��NED���µĺ����ת��Ϊ�����ECEFϵ��������ת�����ڽ�˫GNSS���߲�����Ϣ�������������ת��Ϊ��ECEFϵ�µ�ϵͳ������ֱ�ӹ۲�
%���룺
% heading_n Nϵ�µĺ���� rad
% est_L_b γ�� rad
% est_lambda ���� rad
%�����
% Euler_E=[roll;pitch;yaw] ��ΪECEFϵ rad

Delta_C_n_b=Euler_to_CTM([0,0,heading_n]);
cos_lat = cos(est_L_b);
sin_lat = sin(est_L_b);
cos_long = cos(est_lambda);
sin_long = sin(est_lambda);
C_n_e=[-sin_lat * cos_long, -sin_lat * sin_long,  cos_lat;...
               -sin_long,            cos_long,        0;...
     -cos_lat * cos_long, -cos_lat * sin_long, -sin_lat]';
Delta_C_e_b= C_n_e*Delta_C_n_b*C_n_e';
Euler_E=CTM_to_Euler(Delta_C_e_b);

end

