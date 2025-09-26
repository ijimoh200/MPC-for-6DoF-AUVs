% Generates plots for the Dubins trajectory
select_plts = 2; % plots selector for the Dubins trajectory

x   = 0.5;         y       = 5;          z     = -3.9;
phi = 0;       theta = 0;      psi = 0;
u = 0;         v = 0;          w = 0;
p = 0;         q = 0;          r = 0;

eta_ini     = [x y z phi theta psi]';
nv_ini      = [u v w p q r]';
% Note nv is used to denote \nu

% Sampling and simulation parameters
Ts          = 0.1;
Tf          = 480;
% Tf          = 125;%150;
NoS         = round(Tf/Ts);

%% Generate reference trajectory, noise, current and wave signals
lambda      = 0.2573;
w0          = 0.8;
a           = 0.15;                   % standard deviation
b           = 0;                      % mean of random signal
Kw          = 1.5;
randn('state',80)                     % set the state of randn
T           = 1;       
dt          = T/NoS;
dW          = sqrt(dt)*randn(1,NoS);    % increments for di implementation
W           = cumsum(dW);             % cumulative sum or di implementation
% memory allocation
noise       = zeros(12,NoS);
yrefi        = zeros(6,7);
tau_wave    = zeros(6,NoS);
x_w         = zeros(2, NoS);

%ocean current velcoties are defined as follows:
uc = 0.6; vc = 0.6; wc = 0.4;
% The ocean current is given by
nu_c  = zeros(6,NoS);

Ts = 0.1; % Sampling time (seconds)
k_max = Tf/Ts;% 6000; % Number of time steps (since Ts = 0.02 and duration = 120 seconds)
yrefi = zeros(6, k_max); % Pre-allocate yref for efficiency
time = (0:k_max-1) * Ts; % Time vector

scale_factor = 4; % Scale factor for enlarging the path size

for k = 1:k_max
    t = 1/4*k*Ts;
    if t <= 20
        yrefi(1, k) = 0;
        yrefi(2, k) = 1 * scale_factor;
        yrefi(3, k) = -(0.2 * t + 1) * scale_factor;
        yrefi(6, k) = 0;
    elseif t <= 40
        yrefi(1, k) = (0.2 * (t - 20)) * scale_factor;
        yrefi(2, k) = 1 * scale_factor;
        yrefi(3, k) = -5 * scale_factor;
        yrefi(6, k) = 0;
    elseif t <= 60
        yrefi(1, k) = (sin(0.05 * pi * (t - 40)) + 4) * scale_factor;
        yrefi(2, k) = (-cos(0.05 * pi * (t - 40)) + 2) * scale_factor;
        yrefi(3, k) = -5 * scale_factor;
        yrefi(6, k) = (0.05 * pi * (t - 40)) ;%* scale_factor;
    elseif t <= 80
        yrefi(1, k) = (-0.2 * (t - 60) + 4) * scale_factor;
        yrefi(2, k) = 3 * scale_factor;
        yrefi(3, k) = -5 * scale_factor;
        yrefi(6, k) = pi ;%* scale_factor;
    elseif t <= 100
        yrefi(1, k) = (-sin(0.05 * pi * (t - 80))) * scale_factor;
        yrefi(2, k) = (-cos(0.05 * pi * (t - 80)) + 4) * scale_factor;
        yrefi(3, k) = -5 * scale_factor;
        yrefi(6, k) = (pi - 0.05 * pi * (t - 80));% * scale_factor;
    elseif t <= 120
        yrefi(1, k) = (0.2 * (t - 100)) * scale_factor;
        yrefi(2, k) = 5 * scale_factor;
        yrefi(3, k) = -5 * scale_factor;
        yrefi(6, k) = 0 * scale_factor;
    else
        % Extend behavior beyond t=120 as required
    end

     wn = a.*randn(1,1) + b;
    x_w(:,k+1) = [0 1; -w0^2 -2*lambda*w0]*x_w(:,k) + [0; Kw]*wn;
    tau_wave(1:3,k) = [0 1]*x_w(:,k) + 50*W(:,k);

    linearposition_noise   = (0.005).*rand(3,1);
    angularposition_noise  = (5.42*10^(-5)).*rand(3,1);
    linearvelocity_noise   = (0.1).*rand(3,1);
    angularvelocity_noise  = (5.42*10^(-3)).*rand(3,1);
    noise(:,k)  = [linearposition_noise;angularposition_noise; ...
        linearvelocity_noise;angularvelocity_noise];
end

yref = yrefi;

% Computation of R_a
delta_xd = diff(yref(1,:));
delta_yd = diff(yref(2,:));
delta_zd = diff(yref(3,:));
N_d = Tf;
% Compute the sums of absolute values inside the equation
sum_x = sum(abs(delta_xd));
sum_y = sum(abs(delta_yd));
sum_z = sum(abs(delta_zd));
% Calculate the parameter R_a using the given formula
R_a = (1 / N_d) * sqrt((sum_x)^2 + (sum_y)^2 + (sum_z)^2);
% Display the result
fprintf('The value of R_a is: %.4f\n', R_a);

% plot trajectory
figure
subplot(221)
plot3(yref(1, :), yref(2, :), yref(3, :),'--k')

subplot(222)
plot(yref(1, :), yref(2, :),'--k')

subplot(223)
plot(yref(2, :), yref(3, :),'--k')

subplot(224)
plot(yrefi(1, :), yrefi(3, :),'--k')
