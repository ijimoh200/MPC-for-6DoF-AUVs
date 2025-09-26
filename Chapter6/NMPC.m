function [ui, xi, yi, t_c, yrefcalc, tfinal] = NMPC(eta_ini,nv_ini,yref,nu_c,Ts,Tf,tau_wave)
%This function implements the LPV-MPC2 proposed in this PhD research

NoS         = round(Tf/Ts);
nx          = 12;
nu          = 6;
ny          = 6;
N           = 12;
Nu          = 2;

J = zeros(1, NoS);
t_c = zeros(1,NoS);
yrefcalc = zeros(3,NoS);

tfinal = 0;

% State and input constraint

nlobj = nlmpc(nx, ny, nu);

%%
% The prediction model has a sample time of |0.1| seconds, which is the
nlobj.Ts = Ts;

%%
% Set the prediction horizon to |10|, which is long enough to capture major
% dynamics in the plant but not so long that it hurts computational
% efficiency. 
nlobj.PredictionHorizon = N;
nlobj.ControlHorizon = Nu;

nlobj.Model.StateFcn = "pred_nmpc";

% To use a discrete-time model, set the |Model.IsContinuousTime| property
% of the controller to |false|.
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = 1;

% output function is defined in the |pendulumOutputFcn| function.
nlobj.Model.OutputFcn = 'AUVOutputFcn';

% It is best practice to provide analytical Jacobian functions whenever
% possible, since they significantly improve the simulation speed. In this
% example, provide a Jacobian for the output function using an anonymous
% function.
nlobj.Jacobian.OutputFcn = @(x,u,Ts) [eye(6) zeros(6)];
nlobj.Weights.OutputVariables = [1, 1, 1, 1, 1, 1]*600;
nlobj.Weights.ManipulatedVariables = [1, 1, 1, 1, 1, 1]*1;

for ct = 1:nu
    nlobj.MV(ct).Min = -1000;
    nlobj.MV(ct).Max = 1000;
end
% for ct = 1:nu
%     nlobj.ManipulatedVariables(ct).RateMin =  -500*Ts;
%     nlobj.ManipulatedVariables(ct).RateMax =  500*Ts;
% end


x0 = [eta_ini;nv_ini];
u0 = zeros(6,1);
validateFcns(nlobj,x0,u0,[],{Ts});

nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};

xHistory = x0;
uHistory = u0;

xk = x0;
mv = u0;

 for k = 1:NoS-1 
    tic
    % Compute optimal control moves.
    [mv,nloptions,info] = nlmpcmove(nlobj,xk,mv,yref(:,k)',[],nloptions);
    t_c(:,k) = toc;
    % Implement first optimal control move and update plant states.
    xk = solver_RK(xk,Ts,mv,0*nu_c(:,k),1*tau_wave(:,k));
    % Save plant states for display.
    xHistory = [xHistory xk]; %#ok<*AGROW>
    uHistory = [uHistory mv];  
 end



xi = xHistory;

yi = xHistory(1:6,:);

ui = uHistory;

xd_ = yrefcalc;

el_t = 0;

