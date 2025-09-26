function [ui, xi, yi, ctime, tfinal, yrefcalc] = MM_MPC_Conf(eta_ini,nv_ini,yref,Ts,Tf,tau_wave)
% The min-max MPC algorithm as presented in the 15th IFAC CAMS conference
% 2024, Virginia, USA.
% Jimoh, I.A. and Yue, H., 2024. Path-following model predictive control
% for a coupled autonomous underwater vehicle. IFAC-PapersOnline.

NoS             = round(Tf/Ts);
nx              = 6;
nu              = 6;
ny              = 6;
Q               = blkdiag(5, 5, 5, .3, .1, .1)*1;
R               = blkdiag(30, 30, 30, 30, 30, 40)*0.9;
N               = 10;
Nu              = 2;

% Memory Locations
x               = zeros(nx+nu,NoS); % Actual state
x(:,1)          = [eta_ini; nv_ini];
y               = zeros(ny,NoS);    % Outputs
ueta            = zeros(nu,NoS);
Dueta           = zeros(nu,NoS);
tau             = zeros(nu,NoS);    % forces and moments applied to vehicle
u_              = zeros(nu, 1);     % ui(k-1) i.e., previous computed velocity
x_1              = x(1,1);
tau_            = zeros(nu, 1);     % input forces at time step k-1
s = 1;
yref_ = x(1:6,1);% yref(:,s);
tfinal = 0;
yrefcalc = zeros(3,NoS);
Vref = zeros(ny,NoS);
ctime = zeros(2,NoS);

d_upper = 0.05*[ones(ny/2,1); ones(ny/2,1)];


xmax =  [inf; inf; inf; inf; 2*pi/5; inf; 1.5; 1; 1; inf; inf; inf];
% xmax =  1*[inf; inf; inf; inf; pi/2.01; inf; inf; inf; inf; inf; inf; inf];
%xmax =  [2.3; 1.5; 0; inf; pi/2.01; inf; 1.5; 1; 0.5; inf; inf; inf];
xmin = -xmax;
tau_max = 4000*[1 1 1 1 1 1]';
tau_min = -tau_max;
Xmax = kron(ones(N,1),xmax);
Xmin = kron(ones(N,1),xmin);
Taumax = kron(ones(Nu,1),tau_max);
Taumin = kron(ones(Nu,1),tau_min);

for k = 1:NoS
    x(:,k) = awgn([x(1:6,k); u_],100);   % Initialise augmented state
    y(:,k) = [eye(nx,nu) zeros(nu)]*x(:,k); % Get initial state measurements
    d_p = kron(ones(N,1),abs(d_upper));
    tic
    Ra =.5;
    if (x(1,k)-yref(1,s))^2 + (x(2,k)-yref(2,s))^2 + (x(3,k)-yref(3,s))^2 < (Ra)^2
        yref_ = yref(:,s);
        s = s+1;
    else
        s = s;
    end
    if (x(1,k)-yref(1,4))^2 + (x(2,k)-yref(2,4))^2 + (x(3,k)-yref(3,4))^2 < (Ra)^2
        tfinal = k;
        break
    else
        tfinal = k;
    end
    [yrefcalc(:,k), phi_d,theta_d] = localrajectory_1(x(:,k), yref(:,s),yref_,Ra);

    Vref(:,k) = [yrefcalc(:,k);0;theta_d;phi_d];

    % Update AUV model
    [~,M, Jk, Ck, Dk, gk] = Naminow_AUV(x(:,k),tau_,0*tau_wave(:,k));

    Sigma =kron(ones(Nu,1), Ck*x(7:12,k) + Dk*x(7:12,k)+gk);
    E = kron(eye(Nu), M/Ts) ;

    yref_p = kron(ones(N,1), Vref(:,k)); %yref_p = kron(ones(N,1), yref(:,k));    % Trajectory prediction
    Aeta = [eye(6) Jk*Ts; zeros(6) eye(6)];
    Beta = [Jk*Ts; eye(6)];
    Geta = [eye(6) zeros(6)];
    S = Q;

    % Contruction of prediction model matrices
    Aeta_p = predA(Geta,Aeta,N);
    Aeta_c = predA1(Aeta,N);
    Beta_p = predB(Geta,Aeta, Beta, N, Nu);
    Beta_c = predB1(Aeta, Beta, N, Nu);
    Q_p = predQ(Q,S,N);
    R_p = kron(eye(Nu), R);

    tic
    H = [Beta_p'*Q_p*Beta_p + R_p Beta_p'*Q_p; Q_p*Beta_p Q_p];
    c1 = 2*Beta_p'*Q_p*(Aeta_p*x(:,k) - yref_p);
    c2 = 2*Q_p*(Aeta_p*x(:,k) - yref_p);
    f = [c1; c2];
    Aqp = [Beta_c eye((nx+ny)*N,ny*N);
        -Beta_c -eye((nx+ny)*N,ny*N);
        E zeros(nu*Nu,ny*N);
        -E zeros(nu*Nu,ny*N);
        zeros((ny)*N, nu*Nu) eye((ny)*N,ny*N);
        zeros((ny)*N, nu*Nu) -eye((ny)*N,ny*N)];
    tau_p = kron(ones(Nu,1),tau_);
    x_p = kron(ones(N,1),x(:,k));
    bqp = [Xmax-Aeta_c*x(:,k);
        -Xmin+Aeta_c*x(:,k);
        Taumax - Sigma;
        -Taumin + Sigma;
        d_p;
        d_p];
    options = optimset('Display', 'off');
    H=(H+H')/2; % To impose symmetry in the Hessian matrix
    fun = @(z)(z'*H*z + f'*z);
    ctime1 = toc;    V0 =  zeros(Nu*nu+N*(ny),1);
    tic
    [V,fval] = fminimax(fun,V0,Aqp,bqp,[],[],[],[],[],options);
    ctime2 = toc;
    ctime(:,k) = [ctime1; ctime2];
    Dueta(:,k) = V(1:nu);
    ueta(:,k) = Dueta(:,k) + u_;
    vdot = Dueta(:,k)/Ts;
    tau(:,k) = M*vdot + Ck*u_ + Dk*u_ + gk;
    % Apply input forces to the AUV dynamic
    x_next = solver_RK(x(:,k),Ts,tau(:,k),tau_wave(:,k));
    x(:,k+1) = x_next;
    u_ = ueta(:,k);  % Updates the previous control input
    x_1 = yrefcalc(1,k);
    tau_ = tau(:,k);
    sprintf('Iteration number = %d of %d',k, NoS)
end

ui = tau;

xi = x;

yi = y;

end


