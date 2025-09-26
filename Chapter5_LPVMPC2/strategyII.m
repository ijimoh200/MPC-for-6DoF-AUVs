function [ui, xi, yi, e_l, e_a,y_ref] = strategyII(eta_ini,nv_ini,yref,nu_c,Ts,Tf,N,tau_wave)
% The MPC algorithm presented here is that by Zhang et al. 2019 which
% relies on the partial velocity form of the kinematic model
% The function outputs inputs, states and outputs as well as the equivalent
% velocities give position measurement.

NoS             = round(Tf/Ts);
nx              = 6;
nu              = 6;
ny              = 6;
Q               = blkdiag(1, 1, 1, 1, 1, 1)*1;
R               = blkdiag(1, 1, 1, 1, 1, 1)*20;
Nu              = N/5;

% Memory Locations
x               = zeros(nx+nu,NoS); % Actual state
x(:,1)          = [eta_ini; nv_ini];
y               = zeros(ny,NoS);    % Outputs
ueta            = zeros(nu,NoS);                 
Dueta           = zeros(nu,NoS);   
tau             = zeros(nu,NoS);    % forces and moments applied to vehicle
u_              = zeros(nu, 1);     % ui(k-1) i.e., previous computed velocity   
tau_            = zeros(nu, 1);     % input forces at time step k-1
e_l = zeros(1,NoS);
e_a = zeros(1,NoS);

 for k = 1:NoS

    x(:,k) = awgn([x(1:6,k); u_],60);   % Initialise augmented state
    y(:,k) = [eye(nx,nu) zeros(nu)]*x(:,k); % Get initial state measurements   
    %Compute error signals for AUV motion
    e_l(:,k) = sqrt((yref(1, k)-y(1, k))^2 + (yref(2, k)-y(2, k))^2+(yref(3, k)-y(3, k))^2);
    e_a(:,k) = sqrt((yref(4, k)-y(4, k))^2 + (yref(5, k)-y(5, k))^2+(yref(6, k)-y(6, k))^2);
    % Update AUV model
    [~,M, Jk, Ck, Dk, gk] = Naminow_AUV(x(:,k),tau_,1*nu_c,0*tau_wave(:,k));
       
    yref_p = kron(ones(N,1), yref(:,k));    % Trajectory prediction
    Aeta = [eye(6) Jk*Ts; zeros(6) eye(6)];
    Beta = [Jk*Ts; eye(6)];    
    Geta = [eye(6) zeros(6)];
    S = 1*Q;

    % Contruction of prediction model matrices
    Aeta_p = predA(Geta,Aeta,N);
    Aeta_c = predA1(Aeta,N);
    Beta_p = predB(Geta,Aeta, Beta, N, Nu);
    Beta_c = predB1(Aeta, Beta, N, Nu);
    Q_p = predQ(Q,S,N);
    R_p = kron(eye(Nu), R);
    H = 2*(Beta_p'*Q_p*Beta_p + R_p);
    f = 2*Beta_p'*Q_p*(Aeta_p*x(:,k) - yref_p);
    % Constraints implementation
    xmax =  [inf; inf; inf; inf; pi/2.01; inf; 1.5; 1; 0.5; inf; inf; inf];
    xmin = -xmax;
    Xmax = kron(ones(N,1),xmax);
    Xmin = kron(ones(N,1),xmin);
    Aqp = [Beta_c; -Beta_c];
    bqp = [Xmax-Aeta_c*x(:,k); -Xmin+Aeta_c*x(:,k)];
    
    % Solve quadratic program
    H=(H+H')/2; % To impose symmetry in the Hessian matrix
    options = optimset('Display', 'off');
    V = quadprog(H, f, Aqp,bqp,[],[],[],[],[],options);    
    Dueta(:,k) = V(1:nu); 
    ueta(:,k) = Dueta(:,k) + u_;
    vdot = Dueta(:,k)/Ts;

    tau_max = 300*[2 2 2 1 1 1]';
    tau_min = -tau_max;
    tau(:,k) = M*vdot + Ck*u_ + Dk*u_ + gk;
    tau(:,k) = min(tau_max, max(tau_min, tau(:,k)));
        
    % Apply input forces to the AUV dynamics    
    x_next = solver_RK(x(:,k),Ts,tau(:,k),nu_c,tau_wave(:,k));  
    x(:,k+1) = x_next;
    u_ = ueta(:,k);  % Updates the previous control input
    tau_ = tau(:,k);

    % sprintf('Iteration number = %d of %d',k, NoS)

 end

ui = tau;

xi = x;

yi = y;

y_ref = yref;

end


