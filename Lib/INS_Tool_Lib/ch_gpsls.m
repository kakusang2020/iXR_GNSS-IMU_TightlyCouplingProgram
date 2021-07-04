% GPS α����С���˷���⣬ ״̬��Ϊ X Y Z B(�Ӳ�)

function [pos, b, norm_dp, G] = ch_gpsls(pos, b,  sv_pos,  pr)

B1=1;
END_LOOP=100;
%���Ǹ���
n = size(sv_pos, 2);

if n < 4
    return
end
    b0 = b;
    while (END_LOOP > B1)
        % ��õ�ǰλ���������վ�ľ���
        r = vecnorm(sv_pos - pos);
        
        % ���H����
        H = (sv_pos - pos) ./ r;
        H =-H';
        
        H = [H(:,1:3),  ones(n,1)];
        
        dp = ((pr - r) + b0 - b)';
        
        % �����û�����
        delta =  (H'*H)^(-1)*H'*dp;
        pos = pos + delta(1:3);
        b = b + delta(4);

         norm_dp = norm(dp);
        G = H;
        
        END_LOOP = vnorm(delta(1:3));
    end%End of While
end


