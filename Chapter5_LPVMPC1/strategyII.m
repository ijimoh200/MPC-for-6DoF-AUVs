function [ui, xi, yi, u_d] = strategyII(eta_ini,nv_ini,yref,nu_c,noise,Ts,Tf)
% The MPC algorithm presented here is that by Zhang et al. 2019 which
% relies on the partial velocity form of the kinematic model

NoS             = round(Tf/Ts);
nx              = 6;
nu              = 6;
ny              = 6;
Q               = blkdiag(1, 1, 1, 1, 1, 1)*1;
R               = blkdiag(1, 1, 1, 1, 1, 1)*20;
N               = 15;
Nu              = 2;
% Memory Locations
x               = zeros(nx+nu,NoS); % Actual state
x(:,1)          = [eta_ini; nv_ini];
y               = zeros(ny,NoS);    % Outputs
ueta            = zeros(nu,NoS);
Dueta           = zeros(nu,NoS);
tau             = zeros(nu,NoS);    % forces and moments applied to vehicle
u_              = zeros(nu, 1);     % ui(k-1) i.e., previous computed velocity
tau_            = [0 0 0 0 81.3 0]';     % input forces at time step k-1
tau_wave = 0;

for k = 1:NoS
    
    x(:,k) = [x(1:6,k); u_];  % Initialise augmented state
    y(:,k) = [eye(nx,nu) zeros(nu)]*x(:,k);
    [~,M, Jk, Ck, Dk, gk] = Naminow_AUV(x(:,k),tau_,0*nu_c(:,k),tau_wave);
    yref_p = kron(ones(N,1), yref(:,k));    % Trajectory prediction
    Aeta = [eye(6) Jk*Ts; zeros(6) eye(6)];
    Beta = [Jk*Ts; eye(6)];
    Geta = [eye(6) zeros(6)];
    S = Q;
    % Contruction of prediction model matrices
    Aeta_p = predA(Geta,Aeta,N);
    Beta_p = predB(Geta,Aeta, Beta, N, Nu);
    Q_p = predQ(Q,S,N);
    R_p = kron(eye(Nu), R);
    H = Beta_p'*Q_p*Beta_p + R_p;
    f = Beta_p'*Q_p*(Aeta_p*x(:,k) - yref_p);
    % Constraints implementation
    ymax =  [inf; inf; inf; inf; pi/2; inf];
    ymin = -[inf; inf; inf; inf; pi/2; inf];
    Ymax = kron(ones(N,1),ymax);
    Ymin = kron(ones(N,1),ymin);
    % constraint matrix and vectors
    Aqp = [Beta_p; -Beta_p];
    bqp = [Ymax-Aeta_p*x(:,k); -Ymin+Aeta_p*x(:,k)];
    % Solve quadratic program
    options = optimset('Display', 'off');
    H=(H+H')/2; % To impose symmetry in the Hessian matrix
    V = quadprog(H, f, Aqp,bqp,[],[],[],[],[],options);
    Dueta(:,k) = V(1:nu);
    ueta(:,k) = Dueta(:,k) + u_;
    vdot = Dueta(:,k)/Ts;
    tau(:,k) = M*vdot + Ck*u_ + Dk*u_ + gk;
    % Apply input forces to the AUV dynamics
    x_next = solver_RK(x(:,k),Ts,tau(:,k),nu_c(:,k),tau_wave);
    % Add white noise to the state measurement
    x(:,k+1) = [x_next(1:6)+noise(1:6,k); x_next(7:12)+noise(7:12,k)];% [awgn(x_next(1:6),90); awgn(x_next(7:12),70)];
    u_ = ueta(:,k);  % Updates the previous control input
    tau_ = tau(:,k);
%     sprintf('Iteration number = %d of %d',k, NoS)
end

ui = tau;

xi = x;

yi = y;

u_d = ueta;

end