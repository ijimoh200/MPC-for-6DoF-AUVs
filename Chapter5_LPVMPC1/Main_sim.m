%% POSITIONING CONTROL DURING DOCKING BASED ON LPV-MPC1
% Initial values of the state vector
x   = 0;            y       = 0;          z     = 0;
phi = 0;            theta   = 0;          psi   = 0;
u   = 0;            v       = 0;          w     = 0;
p   = 0;            q       = 0;          r     = 0;

eta_ini = [x y z phi theta psi]';
nv_ini  = [u v w p q r]'; % Note nv is used to denote \nu

% Sampling and simulation parameters
Ts      = 0.1;
Tf      = 50;
NoS     = round(Tf/Ts);

%% Generate reference trajectory and noise signals

noise   = zeros(12,NoS);
yref    = zeros(6,NoS);
nu_c = zeros(6,NoS);

for k = 1:NoS
    position_noise   = (0.0005).*rand(6,1);
    velocity_noise   = (0.001).*rand(6,1);
    noise(:,k)  = [position_noise; ...
                    velocity_noise];
    if k <= round(NoS/4)
        yref(:,k) = [0; 0; 0; 0; 0; 0];
    elseif k <= round(NoS/2)
        yref(:,k) = [0.5; 0.5; 0.0; 0; 0; 0];
    else
        yref(:,k) = [0.5; 0.5; 0.0; 0; 0; -20*pi/180]; %For constant currents
        % yref(:,k) = [0.5; 0.5; 0.0; 0; 0; 0]; %For time-varying currents
    end
    % The constant ocean current is given as follows:
    nu_c(:,k) = [0.4 0.3 -0.1 0 0 0]';
    % The time-varying ocean current is given as follows:
%     nu_c(:,k)  = [-0.2; 0.2; 0.1; 0; 0; 0] ...
%         +[0.1*cos(3/20*pi*k*Ts)*cos(0.002*k*Ts);...
%         0.1*sin(3/20*pi*k*Ts); 0.1*sin(0.002*k*Ts)*cos(3/20*pi*k*Ts); ...
%         0; 0; 0];
end


%% Simulate Controllers

%Proposed LPV-MPC1 is as follows:
[tau1, x1, y1] = proposed(eta_ini,nv_ini,yref,nu_c,noise,Ts,Tf);

%Uchihori et al. (2021) MPC algorithm is as follows:
[tau2, x2, y2, du2] = strategyI(eta_ini,nv_ini,yref,nu_c,noise,Ts,Tf);

% The Zhang et al. (2019) MPC is simulated by
[tau3, x3, y3] = strategyII(eta_ini,nv_ini,yref,nu_c,noise,Ts,Tf);


% Plot Results
Plot_Results()

%% The performance of controllers compared based on RMSE/MAE
% Error vectors
error1 = y1 - yref;
error2 = y2 - yref;
error3 = y3 - yref;

% Preallocate result vectors
rmse1 = zeros(6,1); mae1 = zeros(6,1);
rmse2 = zeros(6,1); mae2 = zeros(6,1);
rmse3 = zeros(6,1); mae3 = zeros(6,1);

% Loop through each row (state variable)
for i = 1:6
    % Controller 1
    err1 = error1(i,:); 
    rmse1(i,1) = sqrt(mean(err1.^2));
    mae1(i,1)  = max(abs(err1(1,15/Ts:end)));

    % Controller 2
    err2 = error2(i,:);
    rmse2(i,1) = sqrt(mean(err2.^2));
    mae2(i,1)  = max(abs(err2(1,15/Ts:end)));

    % Controller 3
    err3 = error3(i,:);
    rmse3(i,1) = sqrt(mean(err3.^2));
    mae3(i,1)  = max(abs(err3(1,15/Ts:end)));
end

% Optional: State labels
labels = {'x','y','z','phi','theta','psi'};
% Display results
fprintf('\n--- RMSE Comparison ---\n');
for i = 1:6
    fprintf('%s: Ctrl1 = %.4f, Ctrl2 = %.4f, Ctrl3 = %.4f\n', ...
        labels{i}, rmse1(i), rmse2(i), rmse3(i));
end

fprintf('\n--- MAE Comparison ---\n');
for i = 1:6
    fprintf('%s: Ctrl1 = %.4f, Ctrl2 = %.4f, Ctrl3 = %.4f\n', ...
        labels{i}, mae1(i), mae2(i), mae3(i));
end
