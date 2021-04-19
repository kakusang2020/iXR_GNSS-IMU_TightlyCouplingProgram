% ����ʽ��߲��

function p = ch_triangulate(anchor_pos, p,  pr)

% ��վ����
n = size(anchor_pos, 2);

% ��õ�ǰλ���������վ�ľ���
r = vecnorm(anchor_pos - p);

% ���H����
H = (anchor_pos - p) ./ r;
H =-H';

% �����û�����
p =  p + (H'*H)^(-1)*H'*(pr - r)';


end


