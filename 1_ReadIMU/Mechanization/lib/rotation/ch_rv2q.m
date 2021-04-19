function q = ch_rv2q(rv)  % ��Ч��תʸ��ת��Ϊ�任��Ԫ��
    nm2 = rv'*rv;  % ��תʸ����ģ��
    if nm2<1.0e-8  % ���ģ����С�������̩��չ��ǰ���������Ǻ���
        q0 = 1-nm2*(1/8-nm2/384); 
        s = 1/2-nm2*(1/48-nm2/3840);
    else
        nm = sqrt(nm2);
        q0 = cos(nm/2); 
        s = sin(nm/2)/nm;
    end
    q = [q0; s*rv];

