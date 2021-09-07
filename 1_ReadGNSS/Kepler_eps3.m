function [eps3] = Kepler_eps3(e,M,x)
t1 = cos(x);
t2 = -1+e*t1;
t3 = sin(x);
t4 = e*t3;
t5 = -x+t4+M;
t6 = t5/(0.5*t5*t4/t2+t2);
disp(t1);
disp(t2);
disp(t3);
disp(t4);
disp(t5);
disp(t6);
eps3 = t5/((0.5*t3-1/6*t1*t6)*e*t6+t2);
end