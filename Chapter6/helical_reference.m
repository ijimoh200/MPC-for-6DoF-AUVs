% Generates plots for the Helical trajectory
select_plts = 1; % plots selector for the helical trajectory

% x = -25;       y = 10;          z = -8;
x   = -11;         y       =  1;          z     = -0.7;
phi = 0;       theta = 0;      psi = 0;
u = 0;         v = 0;          w = 0;
p = 0;         q = 0;          r = 0;

eta_ini     = [x y z phi theta psi]';
nv_ini      = [u v w p q r]';
% Note nv is used to denote \nu

% Sampling and simulation parameters
Ts          = 0.1;
Tf          = 600;
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
yref        = zeros(6,7);
yref1       = zeros(6,NoS);
tau_wave    = zeros(6,NoS);
x_w         = zeros(2, NoS);
%ocean current velcoties are defined as follows:
uc = 0.6; vc = 0.6; wc = 0.4;
% The ocean current is given by
nu_c  = zeros(6,NoS);

for k = 1:NoS

    nu_c(:,k)  = 200*[uc*sin(0.1*k*Ts); vc*sin(0.1*k*Ts); wc*cos(0.05*k*Ts); 0; 0; 0];
   
    yref(:,k) = [-10+0.4*k*Ts; 6*sin(0.02*k*Ts); 10*cos(0.02*k*Ts)-10; 0; -atan(0.1/pi); 0.005*pi*k*Ts]; 

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
plot(yref(1, :), yref(3, :),'--k')

