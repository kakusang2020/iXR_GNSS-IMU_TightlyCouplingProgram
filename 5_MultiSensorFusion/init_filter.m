function [P, Q, R, H] = init_filter(settings)
%%  Init filter
% Kalman filter state matrix
P = zeros(15);
P(1:3,1:3) = settings.factp(1)^2*eye(3);
P(4:6,4:6) = settings.factp(2)^2*eye(3);
P(7:9,7:9) = diag(settings.factp(3:5)).^2;
P(10:12,10:12) = settings.factp(6)^2*eye(3);
P(13:15,13:15) = settings.factp(7)^2*eye(3);

% Process noise covariance
Q = zeros(12);
Q(1:3,1:3) = diag(settings.sigma_acc).^2*eye(3);
Q(4:6,4:6) = diag(settings.sigma_gyro).^2*eye(3);
Q(7:9,7:9) = settings.sigma_acc_bias^2*eye(3);
Q(10:12,10:12) = settings.sigma_gyro_bias^2*eye(3);

% GNSS-receiver position measurement noise
R = settings.sigma_gpspos^2*eye(3);

% Observation matrix
H = [eye(3) zeros(3,12)];
end