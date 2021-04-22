
function out = ch_att_upt(in, gyr, dt)

%% Single sample rotation vector
 rv = gyr*dt;
 dq = ch_rv2q(rv);

%% 
%                  dq(1) = 1;
%                  dq(2) = rv(1)*0.5;
%                  dq(3) = rv(2)*0.5;
%                  dq(4) = rv(3)*0.5;

 out = ch_qmul(in, dq);
 out = ch_qnormlz(out);

 %% Update with rotation matrix
% 
%  Cb2n = ch_q2m(in);
%  theta = gyr*dt;
% 
% %C = eye(3) + ch_askew(theta);
% C = ch_rv2m(theta);
% 
% Cb2n = Cb2n * C;
% 
% % truncation error 5.80
% c1 = Cb2n(1,:);
% c2 = Cb2n(2,:);
% c3 = Cb2n(3,:);
% c1 = 2 / (1 + dot(c1,c1))*c1;
% c2 = 2 / (1 + dot(c2,c2))*c2;
% c3 = 2 / (1 + dot(c3,c3))*c3;
% Cb2n = [c1; c2; c3];
% 
% out = ch_m2q(Cb2n);


end

