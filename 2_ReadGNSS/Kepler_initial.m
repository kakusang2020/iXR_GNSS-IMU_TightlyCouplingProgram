function [E0] = Kepler_initial(e,M)
e_2 = e^2;
e_3 = e_2 * e;
cos_M = cos(M);

E0 = M + (-0.5*e_3+e+(e_2+1.5*e_3*cos_M)*cos_M)*sin(M);
end