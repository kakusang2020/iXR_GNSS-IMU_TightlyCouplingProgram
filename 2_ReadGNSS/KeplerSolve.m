function [E] = KeplerSolve(e,M,tol)
% Solve the equation M = E - e * sin(E) for E , with iterative calculations
% If e<1 , the equation has only one solution for E
% Input:
% e: eccentricity of the orbit
% M: mean anomaly
% tol: when dE < tol, stop iteratings
% Output:
% E: eccentric anomaly, solution of the equation

Mnorm = mod(M,2*pi);
E0 = Kepler_initial(e,M);
dE = tol+1;
count = 0;
while(dE > tol)
    E = E0 - Kepler_eps1(e,M,E0);
    dE = abs(E - E0);
    E0 = E;
    count = count + 1;
    if(count == 100)
        disp('failed to converge');
        break;
    end
end

end