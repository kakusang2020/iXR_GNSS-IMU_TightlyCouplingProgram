function pos = ch_multilateration(sv_pos, pos, pr, dim)
%% ��С���˷���߲��
% M x N:  M:ά�� 2 OR 3  N:��վ����
% sv_pos: ��վλ�� M x N 
% pos:  M x 1 
% pr:  α�� N x 1
% dim : 2 or 3 : 2: ��ά��λ  3: ��ά��λ

B1= 0.1;
END_LOOP=100;
sv_num = size(sv_pos, 2);
max_retry = 5;
lpos = pos; % ������һ�ε�λ��

if sv_num < 3
    return
end


while (END_LOOP > B1 && max_retry > 0)
    % ��õ�ǰλ���������վ�ľ���
    r = vecnorm(sv_pos - pos);
    
    % ���H����
    H = (sv_pos - pos) ./ r;
    if dim == 2
        H = [H [0 0 -1]'];
    end
    H =-H';
    
    dp = (pr - r)';
    if dim == 2
        dp = [dp; 0];
    end
    
    % �����û�����
    delta =  (H'*H)^(-1)*H'*dp;
    
    %����в�
    END_LOOP = norm(delta);
    
    %����λ��
    pos = pos + delta;
    max_retry = max_retry - 1;
    
    %����ʧ��
    if(max_retry == 0 && END_LOOP > 10)
        pos = lpos;
        return;
    end
    
end

end


%
% % ��С���˷���߲��
%
% function pos = ch_multilateration(anchor_pos,  pos, pr)
%
% pr = pr(1:size(anchor_pos, 2));
%
% b = vecnorm(anchor_pos).^(2) - pr.^(2);
% b = b(1:end-1) - b(end);
% b = b';
%
% A =  anchor_pos - anchor_pos(:,end);
% A = A(:,1:end-1)'*2;
%
% pos = (A'*A)^(-1)*A'*b;
%
%
% end

