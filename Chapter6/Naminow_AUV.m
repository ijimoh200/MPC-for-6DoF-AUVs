function [xi_dot,M, Jk, Ck, Dk, gk] = Naminow_AUV(xi,ui,nu_c,dw)
% Implementation of kinematic and dynamic models of the Naminow-D AUV
% xi=[x y z phi theta psi u v w p q r] is the state vector
% The matrices (M, Jk, Ck, Dk) and vector (gk) describe the AUV dynamics.
% nu_c is ocean current vector and dw is the ocean wave vector
% Earth-fixed coordinates
% x = Position in x-direction           [m]
% y = Position in y-direction           [m]
% z = Position in z-direction           [m]
% phi = Roll angle                      [rad]
% theta = Pitch angle                   [rad]
% psi = Yaw angle                       [rad]

% Body-referenced Coordinates
% u = Surge velocity                    [m/sec]
% v = Sway velocity                     [m/sec]
% w = Heave velocity                    [m/sec]
% p = Roll rate                         [rad/sec]
% q = Pitch rate                        [rad/sec]
% r = Yaw rate                          [rad/sec]

% Input vector
% ui = [tau_u, tau_v, tau_w, tau_p, tau_q, tau_r]

% Check for dimensional accuracy of inputs state and control to the function
if (length(xi) ~= 12), error('State vector must have a dimension of 12!');
end

if (length(ui) ~= 6), error('Input vector must have a dimension of 6!');
end
% State variables:
x = xi(1);              y = xi(2);                  z = xi(3);
phi = xi(4);            theta = xi(5);              psi = xi(6);
u = xi(7);              v =xi(8);                   w = xi(9);
p = xi(10);             q = xi(11) ;                r = xi(12);
v1 = [u v w]';
v2 = [p q r]';
nu = [v1; v2];
% Input variables                   % Units
tau_u     = ui(1);                  % N
tau_v     = ui(2);                  % N
tau_w     = ui(3);                  % N
tau_p     = ui(4);                  % Nm
tau_q     = ui(5);                  % Nm
tau_r     = ui(6);                  % Nm
% Vehicle's parameter
W       = 1940;                     % N
B       = 1999;                     % N
L       = 3.00;                     % m
Ixx     = 5.8;                      % kg.m^2
Iyy     = 114;                      % kg.m^2
Izz     = 114;                      % kg.m^2
xg      = -1.378;                   % m
yg      = 0;                        % m
zg      = 0.00;                     % m
xb      = xg;                       % m
yb      = yg;                       % m
zb      = zg;                       % m
b       = 0.324;                    % m
rho     = 1024;                     % kg/m^3
g       = 9.8;                      % m/s
m       = W/g;                      % kg
% Hydrodynamic damping coefficients
Xuu     = -12.7;                    % kg/m
Yvv     = -574;                     % kg/m
Zww     = -574;                     % kg/m
Yrr     = 12.3;                     % kg.m/rad^2
Zqq     = 12.3;                     % kg.m/rad^2
Mww     = 27.4;                     % kg
Mqq     = -4127;                    % kg.m^2/rad^2
Nvv     = -27.4;                    % kg
Nrr     = -4127;                    % kg.m^2/rad^2
Kpp     = -0.63;                    % kg.m^2/rad^2
% Added mass coefficients
Xud     = -6;                       % kg
Yvd     = -230;                     % kg
Zwd     = -230;                     % kg
Kpd     = -1.31;                    % kg.m^2/rad
Mqd     = -161;                     % kg.m^2/rad
Nrd     = -161;                     % kg.m^2/rad
Yrd     = 28.3;                     % kg.m^2/rad
Zqd     = -28.3;                    % kg.m^2/rad
Mwd     = -28.3;                    % kg.m
Nvd     = 28.3;                     % kg.m
% Kinematic model:
% Coordinate transform term definition
cos1 = cos(phi); cos2 = cos(theta); cos3 = cos(psi);
sin1 = sin(phi); sin2 = sin(theta); sin3 = sin(psi);
tan2 = tan(theta);
% Rotation matrix
J1 = [cos2*cos3 -sin3*cos1+cos3*sin2*sin1 cos3*cos1*sin2+sin3*sin1;
    sin3*cos2 cos3*cos1+sin1*sin2*sin3 sin2*sin3*cos1-cos3*sin1
    -sin2 cos2*sin1 cos2*cos1];
J2 = [1 sin1*tan2 cos1*tan2;
    0 cos1 -sin1;
    0 (sin1/cos2) (cos1/cos2)];
Jk = [J1 zeros(size(J1,1),size(J2,2));
    zeros(size(J2,1),size(J1,2)) J2 ];
% Dynamic model:
% Added mass matrix
I0 = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];
rgb = [xg yg zg];
MRB11 = m*eye(3);
MRB12 = -m*SS(rgb);
MRB21 = m*SS(rgb);
MRB22 = I0;
MRB = [MRB11 MRB12; MRB21 MRB22];
M11 = [Xud 0 0;
    0 Yvd 0;
    0 0 Zwd];
M12 = [0 0 0;
    0 0 Yrd;
    0 Zqd 0];
M21 = [0 0 0;
    0 0 Mwd;
    0 Nvd 0];
M22 = [Kpd 0 0;
    0 Mqd 0;
    0 0 Nrd];
MA  = [M11 M12; M21 M22];
M = MRB + MA;
% Coriolis and centripetal matrix
CRB = [zeros(3) -m*SS(v1)-m*SS(v2)*SS(rgb); -m*SS(v1)+m*SS(rgb)*SS(v2) -SS(I0*v2)];
CAM = [zeros(3) -SS(M11*v1 + M12*v2);
    -SS(M11*v1 + M12*v2) -SS(M21*v1 + M22*v2)];
Ck = CRB + CAM;
% Hydrodynamic Damping
Dk = -[Xuu*abs(u) 0 0 0 0 0;
    0 Yvv*abs(v) 0 0 0 Yrr*abs(r);
    0 0 Zww*abs(w) 0 Zqq*abs(q) 0;
    0 0 0 Kpp*abs(q) 0 0;
    0 0 Mww*abs(w) 0 Mqq*abs(q) 0;
    0 Nvv*abs(v) 0 0 0 Nrr*abs(r)];
% Buoyancy forces and moments
gk = [(W-B)*sin2;
    -(W-B)*cos2*sin1;
    -(W-B)*cos2*sin1;
    -(yg*W-yb*B)*cos2*sin1 + (zg*W-zb*B)*cos2*cos1;
    (zg*W-zb*B)*sin2 + (xg*W-xb*B)*cos2*cos1;
    -(xg*W-xb*B)*cos2*sin1-(yg*W-yb*B)*sin2];
% Input forces and moments
tau = [tau_u;tau_v;tau_w;tau_p; tau_q;tau_r];
% Compute reltaive velocity wrt unknown ocean current
nu_r = nu - nu_c;
g_x = [Jk*nu_r; -inv(M)*(Ck*nu_r+Dk*nu_r+gk)];
h_u = [zeros(6,6); inv(M)]*tau;
h_w = [zeros(6,6); inv(M)]*dw;
xi_dot = g_x + h_u + h_w;
end