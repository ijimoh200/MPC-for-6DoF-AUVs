select_plts = 1;
% % 3D TRAJECTORY TRACKING AND POINT STABILISATION
% Initial values of the state vector
x = -2;          y = 7.5;          z = 0;
phi = 0;        theta = 0;      psi = pi/5;
u = 0;          v = 0;          w = 0;
p = 0;          q = 0;          r = 0;

eta_ini     = [x y z phi theta psi]';
nv_ini      = [u v w p q r]';
% Note nv is used to denote \nu

% Sampling and simulation parameters
Ts          = 0.1;
Tf          = 500;
NoS         = round(Tf/Ts);

% %% Generate reference trajectory, noise, current and wave signals
lambda      = 0.2573;
w0          = 0.8;
mean           = 0;                   % standard deviation
std           = 0.25;                      % mean of random signal
Kw          = 1.5;
randn('state',80)                     % set the state of randn
T           = 1;       
dt          = T/NoS;
dW          = sqrt(dt)*randn(1,NoS);    % increments for di implementation
W           = cumsum(dW);             % cumulative sum or di implementation

yref        = zeros(6,NoS+1);
yref1       = zeros(6,NoS);
tau_wave    = zeros(6,NoS);
x_w         = zeros(2, NoS);

for k = 1:NoS

    if k<= 0.7*NoS
        yref(:,k) = [10*sin(0.03*k*Ts); 10*cos(0.03*k*Ts);-0.5*k*Ts; 0; 0; pi/6];
    else
        yref(:,k) = [-9.0; -12.5; -175; 0; 0; pi/6];
    end
    wn = normrnd(mean , std);
    x_w(:,k+1) = [0 1; -w0^2 -2*lambda*w0]*x_w(:,k) + [0; Kw]*wn;
    tau_wave(:,k) = [0 1]*x_w(:,k) + 55*W(:,k);
end



% Calculate transition time
Us = 0.15;  % Resultant speed
pt = yref(1:3,NoS*0.7);  % Tail of the smooth trajectory
ps = yref(1:3,end-1); % Docking position
distance = norm(pt - ps);  % Euclidean distance between current point and docking position
ts = round(distance / (Ts * Us));  % Transition time

for k = 1:NoS
    if yref(:, k+1)-yref(:, k) == 0
        m = (k-0.7*NoS);  
        h = m / ts;
        Psdk = pt*(1-h) + ps*h; % Parameterized trajectory
        if h <= 1
            yref(:,k) = [Psdk(1); Psdk(2); Psdk(3); 0; 0; pi/6];
        else
            yref(:,k) = [ps(1); ps(2); ps(3); 0; 0; pi/6];
        end
    end
end

