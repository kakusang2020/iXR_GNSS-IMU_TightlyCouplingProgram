function m = ch_askew(v) 
% ���ɷ��Գƾ���
%
% Input: v - 3x1 vector
% Output: m - v�ķ��Գ���
%                    |  0   -v(3)  v(2) |
%             m = | v(3)  0    -v(1) |
%                    |-v(2)  v(1)  0    |
    m = [ 0,     -v(3),   v(2); 
          v(3),   0,     -v(1); 
         -v(2),   v(1),   0     ];
      