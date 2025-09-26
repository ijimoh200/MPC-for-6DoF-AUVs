function [c,ceq] = circlecon(xo,yo,zo,yref,xd_,xd,slope)
% 
% phi = xd_(4);            theta = xd_(5);              psi = xd_(6);
% cos1 = cos(phi); cos2 = cos(theta); cos3 = cos(psi);
% sin1 = sin(phi); sin2 = sin(theta); sin3 = sin(psi);
% tan2 = tan(theta);
% 
% % Rotation matrix
% J1 = [cos2*cos3 -sin3*cos1+cos3*sin2*sin1 cos3*cos1*sin2+sin3*sin1;
%     sin3*cos2 cos3*cos1+sin1*sin2*sin3 sin2*sin3*cos1-cos3*sin1
%     -sin2 cos2*sin1 cos2*cos1];

kp = 0.2;

%   phi_obs = atan(yo-xd_(2))/(xo - xd_(1))- atan((xd(2)-xd_(2)/(xd(1)-xd_(1))));
line_con = xd(2) - slope*(xd(1)-yref(1)) - yref(2);
      Ro = 4;
c1 = -(xd(1)-xo)^2 - (xd(2)-yo)^2 + (Ro+kp*sqrt((xd(1)-xo)^2 + (xd(2)-yo)^2))^2;
c2 = 0; %[xd(1);xd(2);yref(3)] - xd_(1:3) - J1*v1;
c = [c1; c2];
ceq = [];