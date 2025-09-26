%% 3D PATH FOLLOWING USING MIN-MAX MPC
x = 0;         y = 0;          z = 0;
phi = 0;       theta = 0;      psi = 0;
u = 0;         v = 0;          w = 0;
p = 0;         q = 0;          r = 0;

eta_ini     = [x y z phi theta psi]';
nv_ini      = [u v w p q r]';
% Note nv is used to denote \nu

% Sampling and simulation parameters
Tf          = 300;
Ts          = 0.1;
%  Tf          = 450; %255
NoS         = round(Tf/Ts);

%% Generate reference trajectory, noise, current and wave signals
lambda      = 0.2573;
w0          = 0.8;
a           = 0.15;                   % standard deviation
b           = 0;                      % mean of random signal
Kw          = 1.5;
randn('state',50)                     % set the state of randn
T           = 1;
dt          = T/NoS;
dW          = sqrt(dt)*randn(1,NoS);    % increments for di implementation
W           = cumsum(dW);             % cumulative sum or di implementation

% time = 0:0.1:1000-0.1;
% wiener_process = [time; W];
yref        = zeros(6,4);
yref1       = zeros(6,NoS);
tau_wave    = zeros(6,NoS);
x_w         = zeros(2, NoS);

% Define the coordinates of obstacles
obc = 0; % 0 - No Obstacle Avoidance; 1 - Avoid Obstacle
Poq = [38*obc 52*obc; 27 62; -16 -6.2]; % coordinates of 2 different obstacles

for k = 1:NoS

    yref(1:3,:) = 2*[10 25 35 20; 20 10 25 35; -8 -8 -4 -2];
    wn = a.*randn(1,1) + b;
    x_w(:,k+1) = [0 1; -w0^2 -2*lambda*w0]*x_w(:,k) + [0; Kw]*wn;
    tau_wave(1:3,k) =  [0 1]*x_w(:,k) + 70*W(:,k);

end

% simulate the controller
%Proposed Accelerated min-max LPV MPC algorithm
x0 = [eta_ini;nv_ini];
% Accelerated feedback min-max MPC:
[u1, x1, y1, ctime, tfinal, yrefcalc] = fMM_MPC(x0,yref,Poq,Ts,Tf,tau_wave);
% Open-loop min-max MPC. Version with states in objective function:
%  [u1, x1, y1, ctime, tfinal, yrefcalc] = MM_MPC(x0,yref,Poq,Ts,Tf,tau_wave);
% Conference version of the min-max MPC with output tracking cost function:
% [u1, x1, y1, ctime, tfinal, yrefcalc] = MM_MPC_Conf(eta_ini,nv_ini,yref,Ts,Tf,tau_wave);
% Velocity LPVMPC
% [u1, x1, y1, eta_o, uapprox,tfinal, yrefcalc] = LPVMPC2(eta_ini,nv_ini,yref,Ts,Tf,1*tau_wave,2);

%% Plot results
Plot_results()

%% Performance metrics
% Computational time
building_time = rms(ctime(1,1:tfinal)*1000)
solver_time = rms(ctime(2,1:tfinal)*1000)

% Assuming the following variables are already defined:
N_d = tfinal; % - Number of discrete time steps
eta = x1(1:6,:);% - Actual state vector (6xN_d matrix)
tau = u1;% - Control input vector (6xN_d matrix)
nu = x1(7:12,:); %- Velocity vector (6xN_d matrix)
delta_tau = zeros(6,tfinal);

tau_p = zeros(6,1);

for ct=1:NoS
    delta_tau(:,ct) = (tau(:,ct)-tau_p);  % Input change between two time steps
    tau_p = tau(:,ct);
end


% Preallocate variables
IAE_pos = 0;
IAE_ori = 0;
CT_for = 0;
CT_mom = 0;
EC_tot = 0;

for k = 1:N_d

    % Control effort for forces
    for i = 1:3
        CT_for = CT_for + abs(delta_tau(i, k));
    end

    % Control effort for moments
    for i = 4:6
        CT_mom = CT_mom + abs(delta_tau(i, k));
    end

    % Energy consumption
    for i = 1:6        EC_tot = EC_tot + abs(nu(i, k) * tau(i, k));
    end
end

% Multiply energy consumption by sampling time and convert to Wh
EC_tot = Ts * EC_tot / 3600;

% Display the results
fprintf('IAE for Position: %.4f\n', IAE_pos);
fprintf('IAE for Orientation: %.4f\n', IAE_ori);
fprintf('Control Effort for Forces: %.4f\n', CT_for);
fprintf('Control Effort for Moments: %.4f\n', CT_mom);
fprintf('Total Energy Consumption (Wh): %.4f\n', EC_tot);
