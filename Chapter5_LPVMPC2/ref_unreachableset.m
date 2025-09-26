select_plts = 2;
% Initial values of the state vector
x   = 1.5;            y       = -1;         z     = -10;
phi = 0;            theta   = 0;          psi   = 0;
u   = 0;            v       = 0;          w     = 0;
p   = 0;            q       = 0;          r     = 0;
eta_ini = [x y z phi theta psi]';
nv_ini  = [u v w p q r]'; % Note nv is used to denote \nu
N = 20; %Prediction horizon
% Sampling and simulation parameters
Ts      = 0.1;
Tf      = 600;
NoS     = round(Tf/Ts);

% Generate reference trajectory, noise, current and wave signals
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

% time = 0:0.1:1000-0.1;
% 
% wiener_process = [time; W];

noise   = zeros(12,NoS);
yref    = zeros(6,NoS+1);

tau_wave    = zeros(6,NoS);
x_w         = zeros(2, NoS);
% Generate the wave disturbances
for k = 1:NoS
    wn = normrnd(mean , std);
    x_w(:,k+1) = [0 1; -w0^2 -2*lambda*w0]*x_w(:,k) + [0; Kw]*wn;
    tau_wave(:,k) = [0 1]*x_w(:,k) + 55*W(:,k);
end

% Define the reference points
start_ref = [-0.00; -0.00; z];
mid_ref = [-17.4; -28.8; z];
end_ref = [-12.3; -1; z];

% Time settings
T_mid = Tf/2;    % Time to reach the middle point (half the total time)
T_final = Tf;  % Time to reach the final point
Ts = 0.1;     % Sampling time
t = 0:Ts:Tf-Ts;  % Time vector

% Interpolate from start to mid point
for i = 1:length(t)
    if t(i) <= T_mid
        % Linear interpolation between start and mid
        yref(1:3,i) = start_ref + (t(i)/T_mid) * (mid_ref - start_ref);
        yref(6,i) = -pi/6;
    else
        % Linear interpolation between mid and end
        yref(1:3,i) = mid_ref + ((t(i) - T_mid)/(T_final - T_mid)) * (end_ref - mid_ref);
        yref(6,i) = pi/6;
    end
end



% % Calculate transition time
% Us = 0.15;  % Resultant speed
% ts = round(distance / (Ts * Us));  % Transition time
% pt = yref(1:3,NoS*0.7);  % Tail of the smooth trajectory
% ps = yref(1:3,end-1); % Docking position
% distance = norm(pt - ps);  % Euclidean distance between current point and docking position
% 
% for k = 1:NoS
%     if yref(:, k+1)-yref(:, k) == 0
%         m = (k-0.7*NoS);  
%         h = m / ts;
%         Psdk = pt*(1-h) + ps*h; % Parameterized trajectory
%         if h <= 1
%             yref(:,k) = [Psdk(1); Psdk(2); Psdk(3); 0; 0; pi/6];
%         else
%             yref(:,k) = [ps(1); ps(2); ps(3); 0; 0; pi/6];
%         end
%     end
% end