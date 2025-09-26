function [c,ceq] = con_nonln(state,xo,yo,yref,yref_,xd_,xd,rho_s,vd)

% Coordinate transform term definition
phi = xd_(4); theta = xd_(5); psi = xd_(6);
cos1 = cos(phi); cos2 = cos(theta); cos3 = cos(psi);
sin1 = sin(phi); sin2 = sin(theta); sin3 = sin(psi);
tan2 = tan(theta);

% Rotation matrix
J1 = [cos2*cos3 -sin3*cos1+cos3*sin2*sin1 cos3*cos1*sin2+sin3*sin1;
    sin3*cos2 cos3*cos1+sin1*sin2*sin3 sin2*sin3*cos1-cos3*sin1
    -sin2 cos2*sin1 cos2*cos1];




x = state;
Ro = 6;
Pk = x(1:3);
Gc = [1 0 0; 0 1 0];
phi_d = atan2((yref(2)-x(2)),(yref(1)-x(1)));

dxyd = sqrt((yref(1)-x(1))^2+(yref(2)-x(2))^2);

theta_o = -atan((yref(3)-x(3))/dxyd);

% con_1 = (Gc*(xd-Pk))'*(Gc*(xd-Pk))*(1+(tan(theta_o))^2)-rho_s^2;
con_x = (xd(3)-Pk(3)) - ((tan(theta_o))^2)*((xd(1)-x(1))^2 + (xd(2)-x(2))^2) ;
con_1 = ((xd-Pk))'*(xd-Pk) - rho_s^2;
con_2 = xd(1:3) - xd_(1:3) - J1*vd;
con_3 = -(xd(1)-xo)^2 - (xd(2)-yo)^2 + Ro^2;
con_4 = (xd(2) - x(2)) - (xd(1) - x(1))*tan(phi_d);
pa = sqrt((xd(1) - yref_(1))^2 + (xd(2) - yref_(2))^2);
pb = sqrt((xd(1) - yref(1))^2 + (xd(2) - yref(2))^2);
ab = sqrt((yref(1) - yref_(1))^2 + (yref(2) - yref_(2))^2);
con_5 = pa + pb - ab;


c = [con_1; con_2];
ceq = [con_x; con_4];