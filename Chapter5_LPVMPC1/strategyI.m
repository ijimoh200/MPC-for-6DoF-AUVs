function [ui, xi, yi, du] = strategyI(eta_ini,nv_ini,yref,nu_c,noise,Ts,Tf)
% This function implements the original LPV-MPC  developed for the Naminow-D
%  AUV by Uchihori etl. (2021) 

NoS = round(Tf/Ts);
nx = 12;
nu = 6;
ny = 6;
Q =  blkdiag(1, 1, 1, 1, 1, 1);
R = 0.0020*eye(6);
N = 10;
Nu = 1;

% Memory Locations
x      = zeros(nx,NoS);    % State vector comprising position and velocities
x(:,1) = [eta_ini; nv_ini];
y      = zeros(ny,NoS);
yi      = zeros(ny,NoS);
tau    = zeros(nu,NoS);
du     = zeros(nx, NoS);
tau_   = [0 0 0 0 75 0]';
xhat   = zeros(nx,NoS);
Qn     = eye(nx)*100;
Rn     = eye(ny)*1000;
Pn     = rand(nx, nx);
G = [eye(6) zeros(6)];

for k = 1:NoS

    y(:,k) = G*x(:,k);
    tau_wave = 0; % not considered here
    yref_p = kron(ones(N,1), yref(:,k));    % Trajectory prediction
    [~,M, Jk, Ck, Dk, gk] = Naminow_AUV(x(:,k),tau_,0*nu_c(:,k),tau_wave);
    Ac = [zeros(6) Jk; zeros(6) -inv(M)*(Ck+Dk)];
    Bc = [zeros(6); inv(M)];
    dc = [zeros(6,1); -inv(M)*gk];
    dn = dc*Ts;
    dk = dn + du(:,k);
    Ak = Ac*Ts + eye(size(Ac,1));
    Bk = Bc*Ts;
    Bd = eye(nx);
    S = 1e4*eye(6); 
    % Contruction of prediction model matrices
    A_p = predA(G,Ak,N);
    B_p = predB(G,Ak, Bk, N, Nu);
    Bd_p = predB(G,Ak, Bd, N, N);
    Q_p = predQ(Q,S,N);
    R_p = kron(eye(Nu), R);
    H = B_p'*Q_p*B_p + R_p;
    dk_p = kron(eye(N,1), dk);
    tau_p = kron(eye(Nu,1), tau_);
    f = B_p'*Q_p*(A_p*xhat(:,k) + Bd_p*dk_p - yref_p) - R_p*tau_p;
    % State constraint
    xmax =  [inf; inf; inf; inf; pi/2; inf];
    xmin = -[inf; inf; inf; inf; pi/2; inf];
    Xmax = kron(ones(N,1),xmax);
    Xmin = kron(ones(N,1),xmin);
    Aqp = [B_p; -B_p];
    bqp = [Xmax-A_p*xhat(:,k)-Bd_p*dk_p; -Xmin+A_p*xhat(:,k)+Bd_p*dk_p];
    % Solve quadratic program
    options = optimset('Display', 'off');
    H=(H+H')/2; % To impose symmetry in the Hessian matrix
    V = quadprog(H, f, Aqp,bqp,[],[],[],[],[],options);
    tau(:,k) = V(1:nu);
    % Apply input forces to the AUV dynamics
    x_next = solver_RK(x(:,k),Ts,tau(:,k),1*nu_c(:,k),tau_wave);
    % Add white noise to the state measurement
    x(:,k+1) = [x_next(1:6)+noise(1:6,k); x_next(7:12)+noise(7:12,k)]; %[awgn(x_next(1:6),90); awgn(x_next(7:12),70)];
    % LPV Kalman for estimation
    Pn = Ak*Pn*Ak'+Qn;
    L =  Ak*Pn*G'/(G*Pn*G'+Rn);
    xhat(:,k+1) = Ak*xhat(:,k) + Bk*tau(:,k) + Bd*dn + L*(y(:,k) - G*xhat(:,k));
    Pn = (Ak-L*G)*Pn*(Ak-L*G)' + Qn + L*Rn*L';
    du(:,k) = (Ak*x(:,k)+Bk*tau(:,k) + dn) - xhat(:,k+1) ;
    % Updates the previous control input
    tau_ = tau(:,k);
    %sprintf('Iteration number = %d of %d',k, NoS)

end

xi = x;

yi = y;

ui = tau;

end
