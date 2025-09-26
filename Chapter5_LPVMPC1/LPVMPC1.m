function [ui, xi, yi] = LPVMPC1(eta_ini,nv_ini,yref,nu_c,noise,Ts,Tf)
% This function implements the velocity form LPV-MPC1

NoS = round(Tf/Ts);
nx = 6;
nu = 6;
ny = 6;
Q = blkdiag(1, 1, 1, 1, 1, 1)*1000;
R = blkdiag(1, 1, 1, 1,1, 1)*0.002;
N = 15;
Nu = 2;
% Memory Locations
x        = zeros(nx,NoS);  % System states defined as the AUV velocities
x(:,1)   = nv_ini;
y        = zeros(ny,NoS);  % System output defined as the AUV position
y(:,1)   = eta_ini;
tau      = zeros(nu,NoS);  % Input foeces and mements
Dtau     = zeros(nu,NoS);
tau_     = [0 0 0 0 75 0]';   % Control input at time step k-1
xtilde = zeros(nx+ny,NoS); % Augemented state
Dx        = zeros(nx,NoS); % State increment Dx = x(k) - x(k-1)
Dy        = zeros(ny,NoS); % Output increment Dy = y(k) - y(k-1)
tau_wave = 0;              % Defined as zero since ocean wave disturbances
% are not considered
for k = 1:NoS-1
    yref_p = kron(ones(N,1), yref(:,k));    % Trajectory prediction
    % Obtain update AUV matrices based on current state
    [~,M, Jk, Ck, Dk,~] = Naminow_AUV([y(:,k);x(:,k)],tau_,0*nu_c(:,k),tau_wave);
    Ak = eye(6)-inv(M)*(Ck+Dk)*Ts;
    Bk =  inv(M)*Ts;
    Hk = Jk*Ts;
    Atilde = [Ak zeros(nx,ny); Hk eye(ny)];
    Btilde = [Bk; zeros(ny, nu)];
    Gtilde = [zeros(nx) eye(ny)];
    Bdtilde = [zeros(nx); eye(nx)];
%     [X,~,~,~] = dare(Ak,Bk, Q,R,[],[]); % Computes terminal weight based...
    % on discrete-time algebraic Riccati Eqn
    S  = Q;%1*X;
    % Contruction of prediction model matrices
    A_p = predA(Gtilde,Atilde,N);
    B_p = predB(Gtilde,Atilde, Btilde, N, Nu);
    Bd_p = predB(Gtilde,Atilde, Bdtilde, N, N);
    Q_p = predQ(Q,S,N);
    R_p =  kron(eye(Nu), R);
    Dy_p = kron(ones(N,1), Dy(:,k));
    H = B_p'*Q_p*B_p + R_p;
    f = B_p'*Q_p*(A_p*xtilde(:,k) + Bd_p*Dy_p - yref_p);
    % Output constraint
    ymax =  [inf; inf; inf; inf; pi/2; inf]*1;
    ymin = -[inf; inf; inf; inf; pi/2; inf]*1;
    Ymax = kron(ones(N,1),ymax);
    Ymin = kron(ones(N,1),ymin);
    Aqp = [B_p; -B_p];
    bqp = [Ymax-(A_p*xtilde(:,k)-Bd_p*Dy_p); -Ymin+(A_p*xtilde(:,k)-Bd_p*Dy_p)];
    % Solve quadratic program
    options = optimset('Display', 'off');
    H=(H+H')/2; % To impose symmetry in the Hessian matrix
    V = quadprog(H, f, Aqp,bqp,[],[],[],[],[],options);
    Dtau(:,k) = V(1:nu);
    tau(:,k) = Dtau(:,k) + tau_;
    % Apply input forces to the AUV dynamics
    x_next  = solver_RK([y(:,k);x(:,k)],Ts,tau(:,k),nu_c(:,k),tau_wave);
    % Add white noise to the state measurement
    x_next = [x_next(1:6)+noise(1:6,k); x_next(7:12)+noise(7:12,k)];
    x(:,k+1) = x_next(7:12) ;
    y(:,k+1) = x_next(1:6);
    Dy(:,k+1) = y(:,k+1) - y(:,k);
    Dx(:,k+1) = x(:,k+1) - x(:,k);
    xtilde(:,k+1) = [Dx(:,k+1); y(:,k+1)];
    % Updates the previous control input, position and velocities
    tau_ = tau(:,k);
%     sprintf('Iteration number = %d of %d',k, NoS)
end

xi = [y;x];

yi = y;

ui = tau;

end
