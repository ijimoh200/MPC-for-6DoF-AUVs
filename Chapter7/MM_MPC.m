function [ui, xi, yi, ctime, tfinal, yrefcalc] = MM_MPC(x0,yref, Poq,Ts,Tf,tau_wave)
% The MPC algorithm presented here is the robust open-loop min-max 
% controller with the state vector used for prediction. The algorithm 
% solves the standard min-max MPC problem.

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


ctime = zeros(2,NoS); % computational time

% Obstacles
xo1 = Poq(1,1); yo1 = Poq(2,1); zo1 = Poq(3,1);
xo2 = Poq(1,2); yo2 = Poq(2,2); zo2 = Poq(3,2);
r_s = 6;    % Active sensing range of onboard sensor
rho_s = 0.6; % switching radius of the sphere of acceptance

V_ = zeros(Nu*nu+N*(ny+ny),1);

for k = 1:NoS
    tic
    % Check condition for switching to the next waypoint
    y(:,k) = [eye(nx,nu) zeros(nu)]*x(:,k); % Get initial state measurements
    tic
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
    [~,M, Jk, Ck, Dk, gk] = Naminow_AUV(x_1,tau_,0*tau_wave(:,k));

    Sigma =kron(ones(Nu,1), Ck*x_1(7:12) + Dk*x_1(7:12)+gk);
    yref_p = kron(ones(N,1), xd_ref(:,k));   % Trajectory prediction
    tic
    % Contruction of prediction model matrices
    Aeta = [eye(6) Jk*Ts; zeros(6) eye(6)];
    Beta = [Jk*Ts; eye(6)];
    Aeta_c = predA1(Aeta,N);
    Beta_c = predB1(Aeta, Beta, N, Nu);
    % Beta_c = Beta_c(1:end,1:Nu*nu);
    H11 = Beta_c'*Q_p*Beta_c + R_p;
    H12 = Beta_c'*Q_p;
    H21 = H12';
    H =[H11 H12; H21 H22];
    h1 = Beta_c'*Q_p*((Aeta_c)*x(:,k)- yref_p);
    h2 = Q_p*(Aeta_c*x(:,k) - yref_p);
    h = [h1; h2];    
    Mbar = kron(eye(Nu), M/Ts);
    Aqp = [Beta_c eye((nx+ny)*N);
        -Beta_c -eye((nx+ny)*N);
        Mbar zeros(nu*Nu,(nx+ny)*N);
        -Mbar zeros(nu*Nu,(nx+ny)*N);
        zeros((nx+ny)*N, nu*Nu) eye((nx+ny)*N);
        zeros((nx+ny)*N, nu*Nu) -eye((nx+ny)*N)];
    tau_p = kron(ones(Nu,1),tau_);
    x_p = kron(ones(N,1),x(:,k));
    bqp = [Xmax-Aeta_c*x(:,k);
        -Xmin+Aeta_c*x(:,k);
        Taumax - Sigma;
        -Taumin + Sigma;
        d_p;
        d_p];
    H=(H+H')/2; % To impose symmetry in the Hessian matrix
    V0 = V_;
    fun = @(z)(z'*H*z + h'*z);
    ctime1 = toc;
    options = optimset('Display', 'off');
    tic
    [V,fval] = fminimax(fun,V0,Aqp,bqp,[],[],[],[],[],options);
    ctime2 = toc;
    ctime(:,k) = [ctime1; ctime2];
    
    Dueta(:,k) = V(1:nu);
    ueta(:,k) = Dueta(:,k) + x_1(7:12);
    vdot = Dueta(:,k)/Ts;
    tau(:,k) = M*vdot + Ck*x_1(7:12) + Dk*x_1(7:12) + gk;
    % Apply input forces to the AUV dynamics
    x_next = solver_RK(x(:,k),Ts,tau(:,k),tau_wave(:,k));
    x(:,k+1) = x_next;
    u_ = ueta(:,k);  % Updates the previous control input
    x_1 = x(:,k);
    tau_ = tau(:,k);
    V_ = V;

    sprintf('Iteration number = %d of %d',k, NoS)

end

ui = tau;

xi = x;

yi = y;

end




