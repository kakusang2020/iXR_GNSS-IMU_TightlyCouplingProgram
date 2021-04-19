%%
%Cartographer SLAM ��̬�㷨����
%���˻�
%����:xiphix@126.com
%΢��:xiphix


function [q] = ch_uv2q(v1, v2)
%Finding quaternion representing the rotation from one vector to another
nv1 = v1/norm(v1);
nv2 = v2/norm(v2);
if norm(nv1+nv2)==0
q = [1 0 0 0]';
else
half = (nv1 + nv2)/norm(nv1 + nv2);
q = [nv1'*half; cross(nv1, half)];
end
end