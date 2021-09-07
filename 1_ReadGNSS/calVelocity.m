function [outputArg1,outputArg2] = calVelocity(s, L, satCoor, ApproCoor, SV_omomi, prn)
eps = 1.0e-25;

a(i) = -(satCoor(1) - ApproCoor(1)) / s;
b(i) = -(satCoor(2) - ApproCoor(2)) / s;
c(i) = -(satCoor(3) - ApproCoor(3)) / s;
vec1(j) = a(i);
vec2(j) = b(i);
vec3(j) = c(i);
aa(i) = a(i) / SV_omomi
a4 = zeros(4,4);
for jj = 1 : 4
    for k =1 : j
        a4(1) = a4(1) + a(i)*a(i);
        a4(2) = a4(2) + a(i)*b(i);
        a4(3) = a4(3) + a(i)*c(i);
        a4(4) = a4(4) + a(i);
        a4(5) = a4(5) + b(i)*a(i);
        a4(6) = a4(6) + b(i)*b(i);
        a4(7) = a4(7) + b(i)*c(i);
        a4(8) = a4(8) + b(i);
        a4(9) = a4(9) + c(i)*a(i);
        a4(10) = a4(10) + c(i)*b(i);
        a4(11) = a4(11) + c(i)*c(i);
        a4(12) = a4(12) + c(i);
        a4(13) = a4(13) + a(i);
        a4(14) = a4(14) + b(i);
        a4(15) = a4(15) + c(i);
        a4(16) = a4(16) + 1;
    end
end
zz = a4;
zz = minv(zz,eps,4);

end

