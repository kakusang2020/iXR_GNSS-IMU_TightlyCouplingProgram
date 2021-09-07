function [esp1] = Kepler_eps1(e,M,x)
esp1 = (x - e * sin(x) - M)/(1 - e * cos(x));
end