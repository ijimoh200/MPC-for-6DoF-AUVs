function [xl, phi_d,theta_d] = localrajectory_1(state, yref,yref_,Ra)
xk = state(1);
yk = state(2);
zk = state(3);
xd = yref(1);
yd = yref(2);
zd = yref(3);
xd_ = yref_(1);
yd_ = yref_(2);
zd_ = yref_(3);
d1 = (yd-yk)/(xd-xk);
a = 1 + d1^2;
g1 = yk - d1*xk;
b = 2*(d1*g1-d1*yk-xk);
c = xk^2+yk^2+g1^2-2*g1*yk-Ra^2;

if abs(xd-xk)>0
    if (xd-xk)>0
        xlos = (-b+sqrt(b^2 - 4*a*c))/(2*a);
        ylos = d1*xlos + g1;
    else
        xlos = (-b-sqrt(b^2 - 4*a*c))/(2*a);
        ylos = d1*xlos + g1;
    end
else
    xlos = xk;
    if (yd-yk)>0
        ylos = yk+sqrt(Ra^2-(xlos-xk)^2);
    else
        ylos = yk-sqrt(Ra^2-(xlos-xk)^2);
    end
end

dxy = sqrt((xd-xk)^2+(yd-yk)^2);

tan_phi_o=atan((zd-zk)/dxy);

% zlos = zd-tan_phi_o*dxy;

zlos = zk+tan_phi_o*sqrt((xlos-xk)^2+(ylos-yk)^2);

xl = [xlos,ylos,zlos]';

if yref(1:3) == yref_(1:3)

    phi_d = 0;

    theta_d = 0;
else
    phi_d = atan2((ylos-yd_),(xlos-xd_));

    dxyd = sqrt((xlos-xd_)^2+(ylos-yd_)^2);

    theta_d = -atan((zlos-zd_)/dxyd);

end





