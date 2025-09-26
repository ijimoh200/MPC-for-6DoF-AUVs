function [ui, xi, yi, dui, yrefcalc,toc_comp] = TMPC(eta_ini,nv_ini,yref,nu_c,Ts,Tf,Ra)
% This function implements the Tube-based LPV-MPC for 3D trajectory
% tracking for AUVs considering incremental input constraints.
NoS = round(Tf/Ts);
nx = 12; % number of states
nu = 6;  % number of inputs
ny = 6;  % number of outputs
Q = blkdiag(1, 1, 1, 1, 1, 1)*2;
R = 0.6*eye(6);
N = 6; % prediction horizon
Nu = 2; % control horizon
N1 = 20; Nu1 = N1; % prediction and control horizon are equal for the SD gain
% Memory Locations
x = zeros(nx,NoS); % State vector comprising position and velocities
x(:,1) = [eta_ini; nv_ini];
x_prev = x(:,1);
y = zeros(ny,NoS);
tau = zeros(nu,NoS);
%tau(:,1) = 1*[0 0 0 0 81.3 0]';
tau_ = tau(:,1);
du = zeros(nu,NoS);
G = [eye(6) zeros(6)];
% constraints definition
xmax = [10000; 10000; 10000; 10000; 2*pi/5; 10000];
xmin = -[10000; 10000; 10000; 10000; 2*pi/5; 10000];
Xmax = kron(ones(N,1),xmax);
Xmin = kron(ones(N,1),xmin);
umax = [1; 1; 1; 1; 1; 1]*1000;
umin = -[1; 1; 1; 1; 1; 1]*1000;
Umax = kron(ones(Nu,1),umax);
Umin = kron(ones(Nu,1),umin);
Au = [eye(Nu*nu); -eye(Nu*nu)];
dumax = [1; 1; 1; 1; 1; 1]*1000*Ts; %100
dumin = -[1; 1; 1; 1; 1; 1]*1000*Ts;
dUmax = kron(ones(Nu,1),dumax);
dUmin = kron(ones(Nu,1),dumin);
Adu = [eye(Nu*nu); -eye(Nu*nu)];
xbar = zeros(nx,NoS);
yrefcalc = yref(1:3,:);
yref_ = yref(:,1);
toc_comp = []; % computational time
V_prev = kron(ones(Nu,1), tau_);

% Simulation loop:
for k = 1:NoS
    y(:,k) = G*x(:,k); % awgn(G*(x(:,k)), 100); % Get initial measurements
    tau_wave = nu_c(:,k); %nu_c = nu_c;
    
    % use the 3D LOS replanning:
    if (x(1,k)-yref(1,k))^2 + (x(2,k)-yref(2,k))^2 + (x(3,k)-yref(3,k))^2 > Ra^2
        yrefcalc(:,k) = localrajectory(x(:,k), yref(:,k),yref_,Ra);
        yref_p = kron(ones(N,1), [yrefcalc(:,k);yref(4:6,k)]); % Trajectory prediction
    else
        yref_p = kron(ones(N,1), yref(:,k)); % Trajectory prediction
    end
    % yref_p = kron(ones(N,1), yref(:,k)); % Trajectory prediction
    [~,M, Jk, Ck, Dk, gk] = Naminow_AUV(x(:,k),tau_,0*nu_c(:,k), 0*tau_wave); %Because disturbance is unknown to the controller
    Ac = [zeros(6) Jk; zeros(6) -inv(M)*(Ck+Dk)];
    Bc = [zeros(6); inv(M)];
    Ak = Ac*Ts + eye(size(Ac,1));
    Bk = Bc*Ts;
    S = Q*1e7;% 10e6*eye(6);
    % Contruction of prediction model matrices
    A_p = predA(G,Ak,N);
    B_p = predB(G,Ak, Bk, N, Nu);
    Q_p = predQ(Q,S,N);
    R_p = kron(eye(Nu), R);
    H = 2*(B_p'*Q_p*B_p + R_p);
    tau_p = kron(ones(Nu,1), tau_);
    f = 2*(B_p'*Q_p*(A_p*x(:,k) - yref_p)) - R_p*tau_p;
    % Solve quadratic program
    tic % state recording computational time
    options = optimset('Display', 'off');
    H=(H+H')/2; % To impose symmetry in the Hessian matrix
    A_pt = predA1(Ak,N1-1);
    B_pt = predB1(Ak, Bk, N1-1, Nu1);
    Q_pt = predQ(eye(12),1e0*eye(12),N1-1);
    R_pt = kron(eye(Nu1), R);
    Ht = 2*(B_pt'*Q_pt*B_pt + R_pt);
    KK = -inv(Ht)*(B_pt'*Q_pt);
    K = KK(end-5:end,:)*A_pt; % State-dependent gain matrix
    phi_x = Ak + Bk*K;
    w_tilde = dist_realisation(phi_x,K,N,N);
    beta = 0.1;
    Zb_state = (1-beta)^(-1)*predB(G,phi_x,eye(nx),N,N)*w_tilde;
    % constraint matrices
    Ax = [B_p; -B_p];
    bx = [Xmax-A_p*x(:,k); -Xmin+A_p*x(:,k)] + [-Zb_state; Zb_state];
    w_tildeU = w_tilde(1:nx*Nu);% dist_realisation(phi_x,K,Nu,Nu);
    Zb_stateU = predB(K,phi_x,eye(nx),Nu,Nu)*w_tildeU; %KKk*Zb_state(1:nx);%
    bu = [Umax; -Umin] + [-Zb_stateU; Zb_stateU];
    bdu = [dUmax; -dUmin] + [tau_p; -tau_p] + [-Zb_stateU; Zb_stateU];
    Aqp = [Au; Adu;Ax];
    bqp = [bu;bdu;bx];
    % implement nonlinear stability constraint for TMPC
    y_prev_p = A_p*x_prev + B_p*V_prev;
    nonlcon=@(V)quadcon(V,V_prev,x(:,k),y_prev_p,yref(:,k),yref_,A_p,B_p,Q,R);
    V = fmincon(@(V)cost(H,f,V),tau_p,Aqp,bqp,[],[],[],[],nonlcon,options);
    V_prev = V;  
    toc_l = toc;
    tau(:,k) = V(1:nu)+ Zb_stateU(1:nu);
    toc_comp = [toc_comp toc_l]; % store computational time
    % Apply input forces to the AUV dynamics
    x(:,k+1) = solver_RK(x(:,k),Ts,tau(:,k),0*nu_c(:,k),tau_wave);
    tauBar = V(1:nu);
    xbar(:,k+1) = Ak*x(:,k) + Bk*tauBar;
    du(:,k) = tau(:,k) - tau_;
    % Updates the previous control input
    tau_ = tau(:,k);
    yref_ = yref(:,k);
    x_prev = x(:,k);
    sprintf('Iteration number = %d of %d',k, NoS)
end
%ui = tau;
xi = x;
yi = y;
ui = tau;
dui = du;
end


% cost function for fmincon
function [J, gradient] = cost(H,f,V)
J = 0.5*V'*H*V + f'*V;
gradient = H\(-f);
end

% Stability constraint
function [c,ceq] = quadcon(V,V_prev,x,eta_prev_p,yref,yref_,A_p,B_p,Q,R)
eta_tilde_1toN= (A_p*x + B_p*V_prev(1:end));
eta_tilde_1toNm2 = eta_tilde_1toN(1:end-12);
eta_bar_km1 = eta_prev_p(1:6);
eta_tilde_Nm1 = eta_tilde_1toN(end-11:end-6);
y_prev_1toNm2 = eta_prev_p(1:end-12);
N = length(eta_prev_p)/6;
Q_pm2 = kron(eye(N-2), Q);
lhs = (eta_tilde_Nm1 - yref)'*Q*(eta_tilde_Nm1 - yref) + V(end-5:end)'*R*V(end-5:end);
%calculate mu
mu = (eta_bar_km1-yref_)'*Q*(eta_bar_km1(1:6)-yref_) + V_prev(1:6)'*R*V_prev(1:6) ...
    + (eta_tilde_1toNm2-y_prev_1toNm2)'*Q_pm2*(eta_tilde_1toNm2-y_prev_1toNm2);
% define constant
epsilon = 0.1;
mu = mu - epsilon;
ceq = [];
c = lhs - mu ;
end
