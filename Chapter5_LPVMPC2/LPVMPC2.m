function [ui, xi, yi, e_l, e_a, dXpred] = LPVMPC2(eta_ini,nv_ini,yref,nu_c,Ts,Tf,N,tau_wave)
%This function implements the LPV-MPC2 proposed in this PhD research

NoS         = round(Tf/Ts);
nx          = 12;
nu          = 6;
ny          = 6;
Nu          = N;

% Memory Locations
x           = zeros(nx,NoS);    
x(:,1)      = [eta_ini; nv_ini];
y           = zeros(ny,NoS);
y(:,1)      = eta_ini;
tau         = zeros(nu,NoS);       
dx          = zeros(nx, NoS);
tau_        = zeros(nu,1);
y_          = y(:,1) ; 
yref_       = yref(:,1); 
Vref        = zeros(ny,NoS);
x_          = zeros(nx, 1); 
G           = [eye(6) zeros(6)];
e_l         = zeros(1,NoS);
e_a         = zeros(1,NoS);
dXpred       = zeros(nx*N,NoS);

J = zeros(1, NoS);

% State and input constraint
xmax =  [inf; inf; inf; inf; 2*pi/5; inf; 1.5; 1; 0.5; inf; inf; 0.04];
xmin = -[inf; inf; inf; inf; 2*pi/5; inf; 1.5; 1; 0.5; inf; inf; 0.04];
tau_max = 600*[2 2 2 1 1 1]';
ymin = -[16; 25; 1000000; 1000000; 2*pi/5; 1000000];
ymax = [16; 25; 1000000; 1000000; 2*pi/5; 1000000];
tau_min = -tau_max;
Xmax = kron(ones(N,1),xmax);
Xmin = kron(ones(N,1),xmin);
Taumax = kron(ones(Nu,1),tau_max);
Taumin = kron(ones(Nu,1),tau_min);

 for k = 1:NoS 
    % Get current measurement
    y(:,k) = awgn(G*x(:,k),60); % add measurement noise
    Q1 = blkdiag(1, 1, 1, 1, 1, 1)*1000;
    Q2 = blkdiag(2, 2, 2, 1, 1, 1)*1000;
    R  = blkdiag(1, 1, 1, 1, 1, 1)*0.05;
    if yref(:, k)-yref_ == 0
        Q = Q2;
    else
        Q = Q1;
    end
    S = 0*Q; % The terminal output has not contribution in the cost i.e. y-yref=0
    options = optimset('Display', 'off');
    Q0 = blkdiag(1, 1, 1, 1, 1, 1);
    H0 = 2*Q0;
    f0 = -2*Q0*yref(:, k); 
    Aiq0 = [-eye(ny); eye(ny)];
    biq0 = [-ymin; ymax];
    Vref(:,k) = quadprog(H0, f0, Aiq0,biq0,[],[],[],[],[],options);
    %Compute error signals for AUV motion
    e_l(:,k) = sqrt((yref(1, k)-y(1, k))^2 + (yref(2, k)...
        -y(2, k))^2+(yref(3, k)-y(3, k))^2);
    e_a(:,k) = sqrt((yref(4, k)-y(4, k))^2 + (yref(5, k)...
        -y(5, k))^2+(yref(6, k)-y(6, k))^2);   
    yref_p = kron(ones(N,1), Vref(:,k));   % Trajectory prediction
    % Update AUV model
    [~,M, Jk, Ck, Dk, gk] = Naminow_AUV(x_,tau_,0*nu_c,0*tau_wave(:,k));
    Ak = [eye(6) Jk*Ts; zeros(6) (eye(6)-inv(M)*(Ck+Dk)*Ts)];
    Bk = [zeros(6); inv(M)*Ts];      
    % Contruction of prediction model matrices
    A_bar = predA1(Ak,N);
    B_bar = predB1(Ak, Bk, N, Nu);
    G_bar = predW(G,N,N);
    I_y = kron(ones(N,1), eye(ny));
    A_p = G_bar*A_bar;
    B_p = G_bar*B_bar;
    Q_p = predQ(Q,S,N);
    R_p = kron(eye(Nu), R);
    H = 2*(B_p'*Q_p*B_p + R_p);
    Xi = I_y*y_;    
    f = 2*B_p'*Q_p*(A_p*dx(:,k) +  Xi - yref_p);
    Aqp = [eye(Nu*nu); -eye(Nu*nu); B_bar; -B_bar];
    tau_p = kron(ones(Nu,1),tau_);
    x_p = kron(ones(N,1),x(:,k));
    bqp = [Taumax-tau_p ; -Taumin + tau_p; Xmax-A_bar*dx(:,k)-x_p; ...
        -Xmin+A_bar*dx(:,k)+x_p];
    % Stability constraint forcing terminal velocities to zeros
    Aeq = B_bar(end-5:end,:);
    beq = -A_bar(end-5:end,:)*dx(:,k);
    % Solve quadratic program
    options = optimset('Display', 'off');
    H=(H+H')/2; % To impose symmetry in the Hessian matrix
    [V,fval] = quadprog(H, f, Aqp,bqp,Aeq,beq,[],[],[],options);
    tau(:,k) = V(1:nu) + tau_;   
    % predictions of state increments
    dXpred(:,k) = A_bar*dx(:,k) + B_bar*V ;
    % Apply input forces to the AUV dynamics
    x(:,k+1) = solver_RK(x(:,k),Ts,tau(:,k),nu_c,tau_wave(:,k));
    dx(:,k+1) = x(:,k+1) - x(:,k);
    y_ = y(:,k);
    x_ = x(:,k);  
    yref_ = yref(:,k);
    tau_ = tau(:,k);
    sprintf('Iteration number = %d of %d',k, NoS)
 end

xi = x;

yi = y;

ui = tau;


