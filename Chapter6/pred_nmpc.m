function xi = pred_nmpc(x,u,Ts)
% RK = Runge Kutta
k1 = NMPC_AUV_pred(x,u);
k2 = NMPC_AUV_pred(x+k1.*(Ts/2),u);
k3 = NMPC_AUV_pred(x+k2.*(Ts/2),u);
k4 = NMPC_AUV_pred(x+k3.*Ts,u);

xi = x + (Ts/6).*(k1+2.*k2+2.*k3 + k4);
end