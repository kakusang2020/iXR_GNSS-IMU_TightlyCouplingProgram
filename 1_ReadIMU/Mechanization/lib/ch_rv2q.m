function q = ch_rv2q(rv)  % Conversion of the equivalent rotation vector to the transformation quaternion
    nm2 = rv'*rv;  % length of model
    if nm2<1.0e-8  % If the modulus is very small, the first few terms of Taylor expansion can be used to find trigonometric functions
        q0 = 1-nm2*(1/8-nm2/384); 
        s = 1/2-nm2*(1/48-nm2/3840);
    else
        nm = sqrt(nm2);
        q0 = cos(nm/2); 
        s = sin(nm/2)/nm;
    end
    q = [q0; s*rv];

