function [wu, Pn, xhat_ppriori ] = kalman_est(x,x_1,xhat,tau,dn,P0,Ak,Bk)
    % % % LPV Kalman for estimation
    nx = 12; % states including position and velocity
    ny = 6; % position variables
    % Define constants
    Qn     = eye(nx)*100;
    Rn     = eye(nx)*1000;
    G = [eye(nx)];
    Bd = eye(nx);
    % Implement estimator
    Pn = Ak*P0*Ak'+Qn;
    L =  Pn*G'/(G*Pn*G'+Rn);
    xhat_apriori = Ak*xhat + Bk*tau + Bd*dn;
    xhat_ppriori = xhat_apriori + L*(x - G*xhat_apriori);
    Pn = (eye(nx)-L*G)*P0*(eye(nx)-L*G)' + L*Rn*L';
    wu = xhat_ppriori - (Ak*x_1+Bk*tau + dn);
end