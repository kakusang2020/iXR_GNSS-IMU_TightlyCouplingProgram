
%%  ('NoiseDensity(�Ƕ��������)', N, 'RandomWalk(�������������)', K,'BiasInstability(��ƫ�ȶ���)', B);
% omega ����Ϊ rad/s
% see Freescale AN: Allan Variance: Noise Analysis for Gyroscopes
function [avar, tau, N, K, B] = ch_allan(omega, Fs, is_plot)

%[avar, tau] = allanvar(omega, 'octave', Fs);

t0 = 1/Fs;
theta = cumsum(omega, 1)*t0;

maxNumM = 200;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2));
m = logspace(log10(10), log10(maxM), maxNumM).';
m = ceil(m); % m must be an integer.
m = unique(m); % Remove duplicates.

tau = m*t0;

avar = zeros(numel(m), 1);
for i = 1:numel(m)
    mi = m(i);
    avar(i,:) = sum( ...
        (theta(1+2*mi:L) - 2*theta(1+mi:L-mi) + theta(1:L-2*mi)).^2, 1);
end
avar = avar ./ (2*tau.^2 .* (L - 2*m));


adev  = sqrt(avar);


%% Angle Random Walk
slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N = 10^logN;

% Plot the results.
tauN = 1;
lineN = N ./ sqrt(tau);



%% Rate Random Walk
slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K = 10^logK;

% Plot the results.
tauK = 3;
lineK = K .* sqrt(tau/3);




%% Bias Instability
slope = 0;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B = 10^logB;

% Plot the results.
tauB = tau(i);
lineB = B * scfB * ones(size(tau));




if is_plot == true
    
    tauParams = [tauN, tauK, tauB];
    params = [N, K, scfB*B];
    figure
    loglog(tau, adev, tau, [lineN, lineK, lineB], '--',     tauParams, params, 'o');
    
    title('Allan Deviation with Noise Parameters')
    xlabel('\tau')
    ylabel('\sigma(\tau)')
    legend('\sigma', '\sigma_N', '\sigma_K', '\sigma_B')
    text(tauParams, params, {'N', 'K', '0.664B'})
    grid on
    axis equal
end

end