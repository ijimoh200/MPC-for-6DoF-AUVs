% This is the main file to run for the tube MPC and NMPC for 3D trajectory
% tracking for an AUV subject to incremental input constraints

% Select the trajectory to simulate:
select = 1; % select = 1 means helical while select = 2 is Dubins
% different radius of acceptance to test its effects on controller
% performance under actuation saturation
if select == 1
    helical_reference()
    Ra = [0.2 0.42 0.65];
elseif select == 2
    dubins_reference()
    Ra = [0.1 0.2 0.4];
end

%% Simulated controllers
% Tube MPC with different radius of acceptance
[tau1, x1, y1, du1, yrefcalc1,toc_linear1] = TMPC(eta_ini,nv_ini,yref,tau_wave,Ts,Tf,Ra(1));
[tau2, x2, y2, du2, yrefcalc2,toc_linear2] = TMPC(eta_ini,nv_ini,yref,tau_wave,Ts,Tf,Ra(2));
[tau3, x3, y3, du3, yrefcalc3,toc_linear3] = TMPC(eta_ini,nv_ini,yref,tau_wave,Ts,Tf,Ra(3));
% Nonlinear MPC:
% [tau4, x4, y4, t_c, yrefcalc4, tfinal4] = NMPC(eta_ini,nv_ini,yref,0*nu_c,Ts,Tf,tau_wave);

% Compute input increment for the NMPC
du4 = zeros(6,NoS);
tau_p = zeros(6,1);
for ct=1:NoS
    du4(:,ct) = (tau4(:,ct)-tau_p);  % Input change between two time steps
    tau_p = tau4(:,ct);
end

%% Plot results
t = 0:Ts:Tf-Ts;
if select_plts == 1
    helical_plots
else
    dubins_plot
end

%% Performance metrics

% The following variables should be defined for each controller:
N_d = NoS; % - Number of discrete time steps
eta = y1;% - Actual state vector (6xN_d matrix)
eta_d = yref;% - Desired state vector (6xN_d matrix)
tau = tau1;% - Control input vector (6xN_d matrix)
nu = x1(7:12,:); %- Velocity vector (6xN_d matrix)
delta_tau = du1;

% Preallocate variables
IAE_pos = 0;
IAE_ori = 0;
CT_for = 0;
CT_mom = 0;
EC_tot = 0;

for k = 1:N_d
    % IAE for position
    for i = 1:3
        IAE_pos = IAE_pos + abs(eta(i, k) - eta_d(i, k));
    end

    % IAE for orientation
    for i = 4:6
        IAE_ori = IAE_ori + abs(eta(i, k) - eta_d(i, k));
    end

    % Control effort for forces
    for i = 1:3
        CT_for = CT_for + abs(delta_tau(i, k));
    end

    % Control effort for moments
    for i = 4:6
        CT_mom = CT_mom + abs(delta_tau(i, k));
    end

    % Energy consumption
    for i = 1:6
        EC_tot = EC_tot + abs(nu(i, k) * tau(i, k));
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
