classdef ch_mahony < handle
    methods (Static = true)
        function q = imu(q, gyr, acc, dt, Kp)
            
            % ���ٶȵ�λ��
            norm_acc =  norm(acc);
            if((norm_acc) == 0), return; end 
            acc = acc / norm(acc);
            
            qtmp = ch_qintg(q, gyr, dt);
            
            v = ch_qmulv(qconj(qtmp), [0 0 1]');
            
            e = cross(acc, v) ;
            
            % ����
            gyr = gyr + Kp * e ;
            
            % ����
            q = ch_qintg(q, gyr, dt);
        end
        
        
        function q = ahrs(q, gyr, acc, mag, dt, Kp)
            
            % ���ٶȼƵ�λ��
            if(norm(acc) == 0), return; end   % handle NaN
            acc = acc / norm(acc);    % normalise magnitude
            
            % �ų���λ��
            if(norm(mag) == 0), return; end    % handle NaN
            mag = mag / norm(mag);   % normalise magnitude
            
            % Reference direction of Earth's magnetic feild
            h = ch_qmulv(q, mag);
            b = [norm([h(1) h(2)]) 0 h(3)]';
            
            % Estimated direction of gravity and magnetic field
            w = ch_qmulv(ch_qconj(q), b);
            v = ch_qmulv(ch_qconj(q), [0 0 1]');
            
            % Error is sum of cross product between estimated direction and measured direction of fields
            e = cross(acc, v) + cross(mag, w);
            
            % Apply feedback terms
            gyr = gyr + Kp * e ;
            
            % integate
            q = ch_qintg(q, gyr, dt);
        end
    end
end