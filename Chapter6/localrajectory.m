function xl = localrajectory(state, yref,yref_,Ra)
% This function implements the 3D LOS local replanner
xk = state(1);
yk = state(2);
zk = state(3);
xd = yref(1);
yd = yref(2);
zd = yref(3);



d1 = (yd-yk)/(xd-xk);
d2 = (zd-zk)/(xd-xk);
a = 1 + d1^2 + d2^2;
g1 = yk - d1*xk;
g2 = zk - d2*xk;
b = 2*(d1*g1+d2*g2-d1*yk-d2*zk-xk);
c = xk^2+yk^2+zk^2+g1^2+g2^2-2*g1*yk-2*g2*zk-Ra^2;

if abs(xd-xk)>0
    if (xd-xk)>0
        xlos = (-b+sqrt(b^2 - 4*a*c))/(2*a);
        ylos = d1*xlos + g1;
        zlos = d2*xlos + g2;
    else
        xlos = (-b-sqrt(b^2 - 4*a*c))/(2*a);
        ylos = d1*xlos + g1;
        zlos = d2*xlos + g2;
    end
else
    alpha = abs(yd-yk)/(abs(yd-yk) + abs(zd-zk));
    xlos = xk;
    if (yd-yk)>0
        ylos = yk+Ra*sqrt(alpha)
    else
        ylos = yk-Ra*sqrt(alpha)
    end
    if (zd-zk)>0
        zlos = zk+Ra*sqrt(1-alpha)
    else
        zlos = zk-Ra*sqrt(1-alpha)
    end
end
xl = [xlos,ylos,zlos]';
