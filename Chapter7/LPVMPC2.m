function [ui, xi, yi, ctime, du,tfinal, yrefcalc] = LPVMPC2(eta_ini,nv_ini,yref,Ts,Tf,tau_wave,select)
% This controller is the velocity form MPC denoted LPVMPC2 in Chapter 5

NoS             = round(Tf/Ts);
nx              = 6;
nu              = 6;
ny              = 6;
Q               = blkdiag(5, 5, 5, .3, .1, .1)*1;
R               = blkdiag(1, 1, 1, 1, 1, 1)*20;
N               = 12;
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
e_l = zeros(1,NoS);
e_a = zeros(1,NoS);

uapprox = zeros(4,NoS);

u_p = zeros(Nu*nu+N*ny,1);

d_upper = 0.1*[1*ones(ny/2,1); 0*ones(ny/2,1)];
d_opt = zeros(nx+nu,1);


% xmax =  [inf; inf; inf; inf; 2*pi/5; inf; 1.5; 1; 1; inf; inf; inf];
xmax =  1*[inf; inf; inf; inf; pi/2.01; inf; inf; inf; inf; inf; inf; inf];
%xmax =  [2.3; 1.5; 0; inf; pi/2.01; inf; 1.5; 1; 0.5; inf; inf; inf];
xmin = -xmax;
tau_max = 20000*[1 1 1 1 1 1]';
tau_min = -tau_max;
Xmax = kron(ones(N,1),xmax);
Xmin = kron(ones(N,1),xmin);
Taumax = kron(ones(Nu,1),tau_max);
Taumin = kron(ones(Nu,1),tau_min);


s = 1;

yref_ = x(1:6,1);% yref(:,s);

tfinal = 0;

J = zeros(1, NoS);

eta_o = zeros(3,NoS);

yrefcalc = zeros(3,NoS);
Vref = zeros(ny,NoS);



% Estimation variables
du     = zeros(nx+nx, NoS);
xhat   = zeros(nx+nx,NoS);
Qn     = eye(nx+nx)*100;
Rn     = eye(ny)*100;
Pn     = rand(nx+nx);
G = [eye(nx,nu) zeros(nu)];

ctime = zeros(2,NoS);

for k = 1:NoS

    x(:,k) = awgn([x(1:6,k); u_],100);   % Initialise augmented state
    tic
    y(:,k) = [eye(nx,nu) zeros(nu)]*x(:,k); % Get initial state measurements
%     d_p = kron(ones(N,1),G*du(:,k));
      d_p = kron(ones(N,1),abs(d_upper));   
    Ra =.6;

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

    if select == 1
        [yrefcalc(:,k), phi_d] = localrajectory(x(:,k), yref(:,s),yref_,Ra);
        %         phi_d = atan2(yref(2,s)-x(2,k),yref(1,s)-x(1,k));
    else
        [yrefcalc(:,k), phi_d,theta_d] = localrajectory_1(x(:,k), yref(:,s),yref_,Ra);
        %         phi_d = atan2(yref(2,s)-x(2,k),yref(1,s)-x(1,k));
    end

    qqqq = yrefcalc(:,k);
    slope = (yref(2,s)-x(2,k))/(yref(1,s)-x(1,k));
    xo = 0*38; yo = 30; zo = -16; doty_o = 0; dotx_o = 0; h = 2; rs = 5;
    Rs=6;
    xo2 = 0*55; yo2 = 60; zo2 = -6.2;

    if (x(1,k)-xo)^2 + (x(2,k)-yo)^2 + (x(3,k)-zo)^2 < (Rs)^2
%         break
        fun = @(xd)1*(xd(1)-qqqq(1))^2/(3^2) + (xd(2)-qqqq(2))^2/(4^2);
        x0 = [qqqq(1),qqqq(2)];
        A = [];
        b = [];
        Aeq = [];
        beq = [];
        lb =  [];
        ub = [];
        nonlcon = @(xd)circlecon(xo,yo,zo,yref(:,s),yref_,xd,slope);
        mmm = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon)';
        Vref(:,k) = [mmm;yrefcalc(3,k);0;theta_d;phi_d];
    elseif (x(1,k)-xo2)^2 + (x(2,k)-yo2)^2 + (x(3,k)-zo2)^2 < (Rs)^2
        fun = @(xd)(xd(1)-qqqq(1))^2*100 + (xd(2)-qqqq(2))^2*(1);
        x0 = [qqqq(1),qqqq(2)];
        A = [];
        b = [];
        Aeq = [];
        beq = [];
        lb =  [];
        ub = [];
        nonlcon = @(xd)circlecon(xo2,yo2,zo2,yref(:,s),yref_,xd,slope);
        mmm = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon)';
        Vref(:,k) = [mmm;yrefcalc(3,k);0;theta_d;phi_d];
    else

        Vref(:,k) = [yrefcalc(:,k);0;theta_d;phi_d];
    end

    % Update AUV model
    [~,M, Jk, Ck, Dk, gk] = Naminow_AUV(x(:,k),tau_,0*tau_wave(:,k));
    % %     M = 0.9*M;
    % %     Ck = 0.9*Ck;
    % %     Dk = 0.9*Dk;

    Sigma =kron(ones(Nu,1), Ck*x(7:12,k) + Dk*x(7:12,k)+gk);
    E = kron(eye(Nu), M/Ts) ;
    % [eye(nu)*M/Ts zeros(nu) zeros(nu);
    % eye(nu)*M/Ts eye(nu)*M/Ts zeros(nu);
    % eye(nu)*M/Ts eye(nu)*M/Ts eye(nu)*M/Ts];




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

    ctime1 = toc;


    % %     Aqp = [E zeros(nu*Nu,ny*N);
    % %         -E zeros(nu*Nu,ny*N);
    % %         zeros((ny)*N, nu*Nu) eye((ny)*N,ny*N);
    % %         zeros((ny)*N, nu*Nu) -eye((ny)*N,ny*N)];
    % %     tau_p = kron(ones(Nu,1),tau_);
    % %     x_p = kron(ones(N,1),x(:,k));
    % %     bqp = [Taumax - Sigma;
    % %           -Taumin + Sigma;
    % %            d_p;
    % %            d_p];

    options = optimset('Display', 'off');
    H=(H+H')/2; % To impose symmetry in the Hessian matrix
    fun = @(z)(z'*H*z + f'*z);
    V0 = u_p*0;
    tic
    [V,fval] = fminimax(fun,V0,Aqp,bqp,[],[],[],[]);
    ctime2 =toc;

    ctime(:,k) = [ctime1; ctime2];




    % %     H = 2*(Beta_p'*Q_p*Beta_p + R_p);
    % %     f = 2*(Beta_p'*Q_p*(Aeta_p*x(:,k) - yref_p));
    % %     % Constraints implementation
    % %     xmax =  [inf; inf; inf; inf; pi/2.01; inf; 1.5; 1; 0.5; inf; inf; inf];
    % %     xmin = -xmax;
    % %     Xmax = kron(ones(N,1),xmax);
    % %     Xmin = kron(ones(N,1),xmin);
    % %     Aqp = [Beta_c; -Beta_c];
    % %     bqp = [Xmax-Aeta_c*x(:,k); -Xmin+Aeta_c*x(:,k)];
    % %
    % %     % Solve quadratic program
    % %     H=(H+H')/2; % To impose symmetry in the Hessian matrix
    % %     options = optimset('Display', 'off');
    % %     V = quadprog(H, f, Aqp,bqp,[],[],[],[],[],options);


    Dueta(:,k) = V(1:nu);
    ueta(:,k) = Dueta(:,k) + u_;
    vdot = Dueta(:,k)/Ts;

    %     tau_max = 600*[2 2 2 1 1 1]';
    %     tau_min = -tau_max;
    tau(:,k) = M*vdot + Ck*u_ + Dk*u_ + gk;
    %     tau(:,k) = min(tau_max, max(tau_min, tau(:,k)));


     %%%%%%%%%%

     % LPV Kalman for estimation
%      [~,M, Jk, Ck, Dk, gk] = Naminow_AUV(x(:,k),tau_,0*nu_c(:,k),tau_wave);
    Ac = [zeros(6) Jk; zeros(6) -inv(M)*(Ck+Dk)];
    Bc = [zeros(6); inv(M)];
    dc = [zeros(6,1); -inv(M)*gk];
    dn = dc*Ts;
    Ak = Ac*Ts + eye(size(Ac,1));
    Bk = Bc*Ts;
    Pn = Ak*Pn*Ak' + Qn;
    L = Pn*G'*(G*Pn*G' + Rn)^(-1);%  Ak*Pn*G'/(G*Pn*G'+Rn);
    dc = [zeros(6,1); -inv(M)*gk];
    dn = dc*Ts;
    Bd = eye(nx+nx);
    xhat(:,k+1) = Ak*xhat(:,k) + Bk*tau(:,k) + Bd*dn + L*(y(:,k) - G*xhat(:,k));
    Pn = (eye(nx+nx) - L*G)*Pn;% (Ak-L*G)*Pn*(Ak-L*G)' + Qn + L*Rn*L';
    du(:,k) = xhat(:,k+1) - (Ak*xhat(:,k)+Bk*tau_ + dn);

    d_upper = G*du(:,k);


    %%%%%%%%%%

    % Apply input forces to the AUV dynamics
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

% end


