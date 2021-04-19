function [p, v, q] = ch_nav_equ_local_tan(p, v, q ,acc, gyr, dt, gN)
%  INS without consider earth rotation
% p          position XYZ  m
% v          velocity XYZ  m/s
% q         Qb2n attitude
% acc      space force  (m/s^2), 
% gyr      gyro (rad/s)]
% dt        dt (s) time interval 0.01s
% gn       g vector

old_v = v;

sf = acc;

%  attitude
q = ch_att_upt(q, gyr, dt);


%  velocity
sf = ch_qmulv(q, sf);
sf = sf + gN;
v = old_v + dt *sf;


%  position
p = p + (old_v + v) *dt/2;

end


% 
% 
% function x = ch_nav_equ_local_tan(x ,u, dt, gN)
% 
% persistent a_old;
% if isempty(a_old)
%    a_old= u(1:3);
% end
%    
% old_v = x(4:6);
% 
% a_new =u(1:3); 
% %sf = sf + 0.5*cross(u(4:6)*dt, sf);
% 
% % attitude
% gyr = u(4:6);
% q_old = x(7:10);
% x(7:10) = ch_att_upt(x(7:10), gyr, dt);
% q_new = x(7:10);
% 
% %  velocity
% 
% x(4:6) = old_v + ((ch_qmulv(q_new, a_new) + ch_qmulv(q_old, a_old) )/2 + gN) *dt;
% 
% %  position
% x(1:3) = x(1:3) + (old_v + x(4:6)) *dt/2;
% a_old = a_new;
% end
% 
% 
