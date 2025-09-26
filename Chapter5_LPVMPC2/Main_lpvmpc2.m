% Select the reference or case study (1 or 2):
case_study = 1;
if case_study == 1
    ref_helicalPLUSdocking % Case study 1
    select_plts = 1;
else
    ref_unreachableset % Case study 2
    select_plts = 2;
end

%ocean current velcoties are defined as follows:
uc = 0.22; vc = 0.18; wc = 0.1;
% The ocean current is given by
nu_c  = [uc; vc; wc; 0; 0; 0];
% tau_wave    = zeros(6,NoS);

%% Simulate Controllers and Plot Results
N = 20; % Prediction horizon
NoS = Tf/Ts;%-1;
t = 0:Ts:NoS*Ts-Ts;
if select_plts == 1
    %Proposed LPV-MPC2 is as follows:
    [tau1, x1, y1, e_l1, e_a1, dXpred] = LPVMPC2(eta_ini,nv_ini,yref,1*nu_c,Ts,Tf,N,tau_wave);
    % The Zhang et al. (2019) MPC is simulated by
    [tau2, x2, y2, e_l2, e_a2] =strategyII(eta_ini,nv_ini,yref,1*nu_c,Ts,Tf,N,tau_wave);
    % Plot results:
    Case1_plots
else
    % Proposed LPVMPC2 with no reachable set (nRS) constraint:
    [tau1, x1, y1, e_l1, e_a1] =LPVMPC_nRS(eta_ini,nv_ini,yref,1*nu_c,Ts,Tf,N,1*tau_wave);
    % Proposed LPV-MPC2 with reachable set constraint enforced:
    [tau2, x2, y2, e_l2, e_a2, dXpred] = LPVMPC2(eta_ini,nv_ini,yref,1*nu_c,Ts,Tf,N, 1*tau_wave);
    % Plot results:
    Case2_plots
end
