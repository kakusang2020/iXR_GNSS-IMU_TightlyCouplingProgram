function [skewMat] = skew(omega,type)
%// Form skew 3x3 skew symmetric matrix from 3x1 vector
if (type == 0)
    skewMat = zeros(3,3);
    skewMat(1) = 0.0;
    skewMat(2) = omega(3);
    skewMat(3) = -omega(2);
    skewMat(4) = -omega(3);
    skewMat(5) = 0.0;
    skewMat(6) = omega(1);
    skewMat(7) = omega(2);
    skewMat(8) = -omega(1);
    skewMat(9) = 0.0;
    %// Form 4x4 skew symmetric matrix from 3x1 vector
elseif (type == 1)
    skewMat = zeros(4,4);
    skewMat(1) = 0.0;
    skewMat(2) = -omega(3);
    skewMat(3) = omega(2);
    skewMat(4) = -omega(1);
    skewMat(5) = omega(3);
    skewMat(6) = 0.0;
    skewMat(7) = -omega(1);
    skewMat(8) = -omega(2);
    skewMat(9) = -omega(2);
    skewMat(10) = omega(1);
    skewMat(11) = 0.0;
    skewMat(12) = -omega(3);
    skewMat(13) = omega(1);
    skewMat(14) = omega(2);
    skewMat(15) = omega(3);
    skewMat(16) = 0.0;
    
    %// Form 4x4 skew symmetric matrix from 4x1 vector
elseif (type == 2)
    skewMat = zeros(4,4);
    skewMat(1) = omega(1);
    skewMat(2) = omega(2);
    skewMat(3) = omega(3);
    skewMat(4) = omega(4);
    skewMat(5) = -omega(2);
    skewMat(6) = omega(1);
    skewMat(7) = omega(4);
    skewMat(8) = -omega(3);
    skewMat(9) = -omega(3);
    skewMat(10) = -omega(4);
    skewMat(11) = omega(1);
    skewMat(12) = omega(2);
    skewMat(13) = -omega(4);
    skewMat(14) = omega(3);
    skewMat(15) = -omega(2);
    skewMat(16) = omega(1);
end
%//Build 3x3 skew and add identity
if (type == 3)
    skewMat = zeros(3,3);
    skewMat(1) = 1.0;
    skewMat(2) = omega(3);
    skewMat(3) = -omega(2);
    skewMat(4) = -omega(3);
    skewMat(5) = 1.0;
    skewMat(6) = omega(1);
    skewMat(7) = omega(2);
    skewMat(8) = -omega(1);
    skewMat(9) = 1.0;
end
end

