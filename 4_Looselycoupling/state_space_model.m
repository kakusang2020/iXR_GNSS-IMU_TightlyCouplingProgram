function [F, G] = state_space_model(x, u, dt)
Cb2n = ch_q2m(x(7:10));

% Transform measured force to force in the tangent plane coordinate system.
sf = Cb2n * u(1:3);
sk = ch_askew(sf);

% Only the standard errors included
O = zeros(3);
I = eye(3);
F = [  O I   O O        O;
    O O -sk -Cb2n O;
    O O O O       -Cb2n;
    O O O O       O;
    O O O O       O];

% 离散化
F = eye(15) + dt*F;

% Noise gain matrix
G=dt*[O       O         O  O;
    Cb2n  O         O  O;
    O        -Cb2n O  O;
    O        O         I   O;
    O        O        O   I];
end