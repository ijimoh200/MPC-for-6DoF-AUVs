function w_tilde = dist_realisation(phi_x,K,N,Nu)
% dist_realisation computes the optimal upper bounds of disturbances used in
% the implementation of the tube-based MPC controller
nx = size(phi_x,1);
B = eye(nx);
nu = size(K,1);
phi_tilde = predB1(phi_x,B,N,N);

wmax = [0*ones(6,1); 0.5*ones(6,1)];
wmin = -wmax;
Wmax = kron(ones(N,1),wmax);
Wmin = kron(ones(N,1),wmin);

A = [eye(N*nx); -eye(N*nx)];
b = [Wmax; Wmax];
Mc  = kron(ones(N,N),[eye(nx); -eye(nx)]);
Nc  = kron(ones(Nu,Nu),[eye(nu); -eye(nu)]*K);
fun = @(x) -(max(Mc*phi_tilde*x) );
w0 = [0*ones(6,1); .01*ones(6,1)]; %ones(nx,1); 
w0_p = kron(ones(N,1),w0);
Alin = [eye(N*nx); -eye(N*nx)];
blin = [Wmax; -Wmin];
lb = Wmin;
ub = Wmax;
M  = [eye(nx); -eye(nx)];
Mpred = kron(eye(N),M);
var = Mpred*phi_tilde;
c = ones(1,size(Mpred,1));
Cost = c*var;% max(var);% c'*var;%  sum(cccc,1);
options = optimset('linprog');
options.Display = 'off';
w_tilde = linprog(Cost,[],[],[],[],lb,ub,options);
end
