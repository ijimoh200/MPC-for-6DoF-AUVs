function [ui, xi, yi, ctime, tfinal, yrefcalc] = fMM_MPC(x0,yref, Poq,Ts,Tf,tau_wave)
% The MPC algorithm presented here is the robust feedback min-max MPC
% controller based on the duality-inspired transformation

% Parameters and Initialization
NoS = round(Tf/Ts);       % Number of samples
nx = 6;                   % Number of states (position variables)
nu = 6;                   % Number of inputs
ny = 6;                   % Number of outputs
Q = blkdiag(4, 4, 4, 0.4, 0.1, 0.1, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01);
% R = blkdiag(22, 22, 22, 22, 22, 30); %obs
%constrained scenario.
R = blkdiag(30, 30, 30, 30, 30, 40)*0.9;
S = Q;                    % State and input weighting matrices
N = 14;                   % Prediction horizon
Nu = 2;                   % Control horizon

% Precompute matrices
Q_p = predQ(Q,S,N);
R_p = kron(eye(Nu), R);
H22 = Q_p;
H22_inv = inv(H22);

% Memory allocations
x               = zeros(nx+nu,NoS); % Actual state comprising position and velocity vectors
x(:,1)          = x0;
y               = zeros(ny,NoS);    % AUV pose (linear and angular position)
ueta            = zeros(nu,NoS);    % Velocity increment
Dueta           = zeros(nu,NoS);    % Acceleration of the vehicle
tau             = zeros(nu,NoS);    % forces and moments applied to vehicle
u_              = zeros(nu, 1);     % ui(k-1) i.e., previous computed velocity
x_1             = x(:,1);           % previous state measurement
tau_            = zeros(nu, 1);     % input forces at time step k-1v
yref_ = x(1:6,1);%                  % Previous waypoint coordinate
tfinal = 0;                         % Record the duration of task
yrefcalc = zeros(3,NoS);
xd_ref = zeros(ny+nx,NoS);          % state reference computed online

% Define constraints
xmax =  [inf; inf; inf; inf; 2*pi/5; inf; 1.5; 1; 0.5; inf; inf; inf];
xmin = -[inf; inf; inf; inf; 2*pi/5; inf; 0; 1; 0.5; inf; inf; inf];
Xmax = kron(ones(N,1),xmax);
Xmin = kron(ones(N,1),xmin);
tau_max = 2000*[1 1 1 1 1 1]';
tau_min = -tau_max;
Taumax = kron(ones(Nu,1),tau_max);
Taumin = kron(ones(Nu,1),tau_min);
s = 1; % first waypoint index


% Disturbance bounds
d_upper = [0.05*ones(nx,1)*1; 0.4*ones(nx,1)];
% d_upper = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, ]
d_p = kron(ones(N,1),abs(d_upper));
D_lower = -(d_p);
D_upper = (d_p);

% Define dual variables
yalmip('clear')
lambda = sdpvar(2*length(D_lower), 1); % Lagrange multipliers for lower bounds
n_lam = 2*length(D_lower);
G2 = [eye(N*(nx+nx)); -eye(N*(nx+nx))];
g2 = [D_upper; -D_lower];
% Define the disturbance D*(k)
D = sdpvar(nx, 1);
U = sdpvar(Nu*nu, 1); % Optimisation variable
ctime = zeros(2,NoS); % computational time

% Obstacles
xo1 = Poq(1,1); yo1 = Poq(2,1); zo1 = Poq(3,1);
xo2 = Poq(1,2); yo2 = Poq(2,2); zo2 = Poq(3,2);
r_s = 6;    % Active sensing range of onboard sensor
rho_s = 0.6; % switching radius of the sphere of acceptance

for k = 1:NoS
    tic
    % Check condition for switching to the next waypoint
    y(:,k) = [eye(nx,nu) zeros(nu)]*x(:,k); % Get initial state measurements
    if (x(1,k)-yref(1,s))^2 + (x(2,k)-yref(2,s))^2 + (x(3,k)-yref(3,s))^2 <= (rho_s)^2
        yref_ = yref(:,s);
        s = s+1;
    else
        s = s;
    end
    % Check condition to terminate simulation
    if (x(1,k)-yref(1,4))^2 + (x(2,k)-yref(2,4))^2 + (x(3,k)-yref(3,4))^2 <= (rho_s)^2
        tfinal = k;
        break
    else
        tfinal = k;
    end
    % Check condition for safe navigation
    if (x(1,k)-xo1)^2 + (x(2,k)-yo1)^2 + (x(3,k)-zo1)^2 <= (r_s)^2
        alpha = 1;
        x_o = xo1;     % obstacle x position
        y_o = yo1;     % obstacle y position
        z_o = zo1;     % obstacle z position
        MO_LOSGS()
        xd_ref(1:6,k) = [xlos; ylos; zlos; 0;theta_d;psi_d];
    elseif (x(1,k)-xo2)^2 + (x(2,k)-yo2)^2 + (x(3,k)-zo2)^2 <= (r_s)^2
        alpha = 1;
        x_o = xo2;     % obstacle x position
        y_o = yo2;     % obstacle y position
        z_o = zo2;     % obstacle z position
        MO_LOSGS()
        xd_ref(1:6,k) = [xlos; ylos; zlos; 0;theta_d;psi_d];
    else
        alpha = 0;
        x_o = xo2*0;     % obstacle x position
        y_o = yo2*0;     % obstacle y position
        z_o = zo1;        % obstacle z position
        MO_LOSGS()
        xd_ref(1:6,k) = [xlos; ylos; zlos; 0;theta_d;psi_d];
    end
    xd_ref(7:12,k) = u_;
    % Update AUV model. Ocean wave is zero because it is unknown to the
    % controller:
    [~,M, ~, Ck, Dk, gk] = Naminow_AUV(x_1,tau_,0*tau_wave(:,k));

    Sigma =kron(ones(Nu,1), Ck*x_1(7:12) + Dk*x_1(7:12)+gk);
    Mbar = kron(eye(Nu), M/Ts);
    yref_p = kron(ones(N,1), xd_ref(:,k));   % Trajectory prediction
    tic
    [Aeta_c, Beta_c] = compute_prediction_matrices(x(1:6,k), x(7:12,k), x_1(7:12), N, Ts);
    Beta_c = Beta_c(1:end,1:Nu*nu);
    H11 = Beta_c'*Q_p*Beta_c + R_p;
    H12 = Beta_c'*Q_p;
    H21 = H12';
    h1 = Beta_c'*Q_p*((Aeta_c)*x(:,k)- yref_p);
    h2 = Q_p*(Aeta_c*x(:,k) - yref_p);
    % Find optimal D
    D_star = -H22_inv * (H12' * U + h2 + G2'*lambda);
    G3 = [Beta_c-H22_inv*H12' -H22_inv*G2';
        -Beta_c+H22_inv*H12' H22_inv*G2';
        Mbar zeros(nu*Nu,n_lam);
        -Mbar  zeros(nu*Nu,n_lam)];
    g3 = [Xmax-Aeta_c*x(:,k) + H22_inv * h2;
        -Xmin+Aeta_c*x(:,k) - H22_inv * h2;
        Taumax - Sigma;
        -Taumin + Sigma];
    G4 = [-G2 * H22_inv * H12', -G2 * H22_inv * G2'];
    g4 = g2 + G2 * H22_inv * h2;
    % Substitute D* into the cost function
    cost = 1/2*[U;D_star]'*[H11 H12; H21 H22]*[U;D_star] +  h1' * U + h2' * D_star;
    % Define constraints
    constraints = [[G3;G4] * [U;lambda] <= [g3;g4]; -lambda <= zeros(2*length(D_upper),1)];
    ctime1 = toc;
    % Solve the minimization problem
    options = sdpsettings('solver','quadprog', 'verbose', 0);
    %options.LargeScale = 'off';
    tic
    sol = optimize(constraints, cost, options);
    ctime2 = toc;
    ctime(:,k) = [ctime1; ctime2];
    % Check for success
    if sol.problem == 0
        % Solution is feasible
        U_opt = value(U);
        D_opt = value(D_star);
        %disp('Optimization successful!');
    else
        % Handle errors
        %disp('Optimization failed!');
    end
    Dueta(:,k) = U_opt(1:nu);
    ueta(:,k) = Dueta(:,k) + x_1(7:12);
    vdot = Dueta(:,k)/Ts;
    tau(:,k) = M*vdot + Ck*x_1(7:12) + Dk*x_1(7:12) + gk;
    % Apply input forces to the AUV dynamics
    x_next = solver_RK(x(:,k),Ts,tau(:,k),tau_wave(:,k));
    x(:,k+1) = x_next;
    u_ = ueta(:,k);  % Updates the previous control input
    x_1 = x(:,k);
    tau_ = tau(:,k);
    %sprintf('Iteration number = %d of %d',k, NoS)
end

ui = tau;

xi = x;

yi = y;

end




