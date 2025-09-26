function xi = solver_RK(x,Ts,u,dw)
% RK = Runge Kutta
k1 = Naminow_AUV(x,u,dw);
k2 = Naminow_AUV(x+k1.*(Ts/2),u,dw);
k3 = Naminow_AUV(x+k2.*(Ts/2),u,dw);
k4 = Naminow_AUV(x+k3.*Ts,u,dw);

xi = x + (Ts/6).*(k1+2.*k2+2.*k3 + k4);
end