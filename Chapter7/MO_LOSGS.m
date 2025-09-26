% Define position variables
k_p = 1.5;    % design gain
x_d = yref(1,s);     % desired x position
y_d = yref(2,s);     % desired y position
z_d = yref(3,s);     % desired z position
xd_ = yref_(1);
yd_ = yref_(2);
zd_ = yref_(3);
x_k = x(1,k);      % current x position
y_k = x(2,k);      % current y position
z_k = x(3,k);      % current z position

% MO-LOS design paramters
r_o = 4; % radius of unsafe region to be avoided
rho_c = 0.6; % Radius of acceptance for 2D LOS computation

% Initial guess for the optimizer
x0 = [x_k, y_k, 0];  % [x_los(k), y_los(k), s_c]

% Objective function
obj_fun = @(x) objective(x, x_d, y_d, x_k, y_k, alpha, rho_c);

% Nonlinear constraints
nonlcon = @(q) constraints(q, yref(:,s), x_k, y_k, x_o, y_o, rho_c, r_o, k_p, alpha);

% Lower and upper bounds for the decision variables
lb = [-Inf, -Inf, 0];  % s_c is non-negative
ub = [Inf, Inf, Inf];

% Solve the optimization problem
options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'interior-point');
[x_opt, fval, exitflag] = fmincon(obj_fun, x0, [], [], [], [], lb, ub, nonlcon, options);

xlos = x_opt(1);
ylos = x_opt(2);

dxy = abs(sqrt((x_d-x_k)^2+(y_d-y_k)^2));

tan_phi_o=atan((z_d-z_k)/dxy);


zlos = z_k+tan_phi_o*abs(sqrt((xlos-x_k)^2+(ylos-y_k)^2));

xl = [xlos,ylos,zlos]';


if yref(1:3) == yref_(1:3)

    psi_d = 0;

    theta_d = 0;
else
    psi_d = atan2((ylos-yd_),(xlos-xd_));


    dxyd = abs(sqrt((xlos-xd_)^2+(ylos-yd_)^2));

    theta_d = -atan((zlos-zd_)/dxyd);

end

function J = objective(x, x_d, y_d, x_k, y_k, alpha, rho_c)
% Extract decision variables
x_los = x(1);
y_los = x(2);
s_c = [x(3)];

% LOS objective
J_los = (x_los )^2 + (y_los )^2;

d1 = (y_d-y_k)/(x_d-x_k);
a = 1 + d1^2;
g1 = y_k - d1*x_k;
b = 2*(d1*g1-d1*y_k-x_k);
c = x_k^2+y_k^2+g1^2-2*g1*y_k-rho_c^2;

if abs(x_d-x_k)>0
    if (x_d-x_k)>0
        xlosc = (-b+abs(sqrt(b^2 - 4*a*c)))/(2*a);
        ylosc = d1*x_los + g1;
    else
        xlosc = (-b-abs(sqrt(b^2 - 4*a*c)))/(2*a);
        ylosc = d1*x_los + g1;
    end
else
    xlosc = x_k;
    if (y_d-y_k)>0
        ylosc = y_k+abs(sqrt(rho_c^2-(x_los-x_k)^2));
    else
        ylosc = y_k-abs(sqrt(rho_c^2-(x_los-x_k)^2));
    end
end

% Obstacle avoidance objective
J_oa = (x_los - xlosc)^2 + (y_los - ylosc)^2 + s_c^2;

% Combined objective function
%     J = ((1/(1+alpha))*J_los + alpha * 0.5 * J_oa);

J = (J_los + alpha * J_oa);
end

function [cin, ceq] = constraints(x, yref, x_k, y_k, x_o, y_o, rho_c, r_o, k_p,alpha)
% Extract decision variables
x_los = x(1);
y_los = x(2);
s_c = [x(3)];
x_d = yref(1);
y_d = yref(2);

% Distance to obstacle
d_o = abs(sqrt((x_k - x_o)^2 + (y_k - y_o)^2));

% Safety constraint slack
epsilon_s = k_p * d_o;

% Nonlinear inequality constraints (c <= 0)
cin = [
    alpha*(-(x_los - x_o)^2 - (y_los - y_o)^2 + (r_o^2 + epsilon_s ));  % obstacle avoidance
    ];

d1 = (y_d-y_k)/(x_d-x_k);
a = 1 + d1^2;
g1 = y_k - d1*x_k;
b = 2*(d1*g1-d1*y_k-x_k);
c = x_k^2+y_k^2+g1^2-2*g1*y_k-rho_c^2;

if abs(x_d-x_k)>0
    if (x_d-x_k)>0
        xlosc = (-b+abs(sqrt(b^2 - 4*a*c)))/(2*a);
        ylosc = d1*x_los + g1;
    else
        xlosc = (-b-abs(sqrt(b^2 - 4*a*c)))/(2*a);
        ylosc = d1*x_los + g1;
    end
else
    xlosc = x_k;
    if (y_d-y_k)>0
        ylosc = y_k+abs(sqrt(rho_c^2-(x_los-x_k)^2));
    else
        ylosc = y_k-abs(sqrt(rho_c^2-(x_los-x_k)^2));
    end
end

% Nonlinear equality constraints (ceq = 0)
ceq = [
    xlosc - x_los;
    ylosc - y_los + alpha * s_c(1);  % soft LOS direction constraint
    ];
end
