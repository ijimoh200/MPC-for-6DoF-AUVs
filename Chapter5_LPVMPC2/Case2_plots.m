
%% plots of the reachable set
RXpred1 = reshape(dXpred(:,3010),[12,N]);
RXpred2 = reshape(dXpred(:,3020),[12,N]);
RXpred3 = reshape(dXpred(:,3030),[12,N]);
RXpred4 = reshape(dXpred(:,3040),[12,N]);
RXpred5 = reshape(dXpred(:,3050),[12,N]);

figure
subplot(321)
plot(RXpred1(7,:), '-*')
hold on
plot(RXpred2(7,:), '-*')
plot(RXpred3(7,:), '-*')
plot(RXpred4(7,:), '-*')
plot(RXpred5(7,:), '-*')
ylabel('$\Delta u$ [m/s]',Interpreter='latex')
xlabel('$j$',Interpreter='latex')
legend('$k=3010$','$k=3020$','$k=3030$','$k=3040$','$k=3050$',...
    'Orientation','horizontal',Interpreter='latex')
ylim([-0.015 0.015])
grid on

subplot(323)
plot(RXpred1(8,:), '-*')
hold on
plot(RXpred2(8,:), '-*')
plot(RXpred3(8,:), '-*')
plot(RXpred4(8,:), '-*')
plot(RXpred5(8,:), '-*')
ylabel('$\Delta v$ [m/s]',Interpreter='latex')
xlabel('$j$',Interpreter='latex')
% ylim([-0.015 0.015])
grid on

subplot(325)
plot(RXpred1(9,:), '-*')
hold on
plot(RXpred2(9,:), '-*')
plot(RXpred3(9,:), '-*')
plot(RXpred4(9,:), '-*')
plot(RXpred5(9,:), '-*')
ylabel('$\Delta v$ [m/s]',Interpreter='latex')
xlabel('$j$',Interpreter='latex')
% ylim([-0.015 0.015])
grid on
subplot(322)
plot(RXpred1(10,:), '-*')
hold on
plot(RXpred2(10,:), '-*')
plot(RXpred3(10,:), '-*')
plot(RXpred4(10,:), '-*')
plot(RXpred5(10,:), '-*')
ylabel('$\Delta p$ [rad/s]',Interpreter='latex')
xlabel('$j$',Interpreter='latex')
% ylim([-0.015 0.015])
grid on

subplot(324)
plot(RXpred1(11,:), '-*')
hold on
plot(RXpred2(11,:), '-*')
plot(RXpred3(11,:), '-*')
plot(RXpred4(11,:), '-*')
plot(RXpred5(11,:), '-*')
ylabel('$\Delta q$ [rad/s]',Interpreter='latex')
xlabel('$j$',Interpreter='latex')
% ylim([-0.015 0.015])
grid on


subplot(326)
plot(RXpred1(12,:), '-*')
hold on
plot(RXpred2(12,:), '-*')
plot(RXpred3(12,:), '-*')
plot(RXpred4(12,:), '-*')
plot(RXpred5(12,:), '-*')
ylabel('$\Delta r$ [rad/s]',Interpreter='latex')
xlabel('$j$',Interpreter='latex')
% ylim([-0.015 0.015])
grid on


%% Plots of velocity variables and input forces and moments
figure
subplot(621)
stairs(t, x1(7, 1:size(t,2)),'-r', 'linewidth', 1.2)
hold on
stairs(t, x2(7, 1:size(t,2)),'--b', 'linewidth', 1.2)
legend('LPVMPC2','MPC3')
ylabel('$u$ [m/s]', 'interpreter','latex')
xlim([0 500])
% ylim([-1.5 1.5])
grid on

subplot(623)
stairs(t, x1(8, 1:size(t,2)),'-r', 'linewidth', 1.2)
hold on
stairs(t, x2(8, 1:size(t,2)),'--b', 'linewidth', 1.2)
ylabel('$v$ [m/s]', 'interpreter','latex')
ylim([-0.2 0.8])
xlim([0 500])
grid on


subplot(625)
stairs(t, x1(9, 1:size(t,2)),'-r', 'linewidth', 1.2)
hold on
stairs(t, x2(9, 1:size(t,2)),'--b', 'linewidth', 1.2)
ylabel('$w$ [m/s]', 'interpreter','latex')
ylim([-0.5 0.3])
xlim([0 500])
grid on


subplot(627)
stairs(t, x1(10, 1:size(t,2)),'-r', 'linewidth', 1.2)
hold on
stairs(t, x2(10, 1:size(t,2)),'--b', 'linewidth', 1.2)
ylabel('$p$ [rad/s]', 'interpreter','latex')
ylim([-0.6 0.6])
xlim([0 500])
grid on


subplot(629)
stairs(t, x1(11, 1:size(t,2)),'-r', 'linewidth', 1.2)
hold on
stairs(t, x2(11, 1:size(t,2)),'--b', 'linewidth', 1.2)
ylabel('$q$ [rad/s]', 'interpreter','latex')
ylim([-0.1 0.1])
xlim([0 500])
grid on

subplot(6,2,11)
stairs(t, x1(12, 1:size(t,2)),'-r', 'linewidth', 1.2)
hold on
stairs(t, x2(12, 1:size(t,2)),'--b', 'linewidth', 1.2)
ylabel('$r$ [rad/s]', 'interpreter','latex')
xlabel('Time [s]')
ylim([-0.12 0.06])
xlim([0 500])
grid on

subplot(622)
stairs(t, tau1(1, 1:size(t,2)),'r', 'linewidth', 1.2)
hold on
stairs(t, tau2(1, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, 600*ones(1,NoS),'-.g')
stairs(t, -600*ones(1,NoS),'-.g')
ylim([-650 650])
ylabel('$\tau_X$ [N]', 'interpreter','latex')

subplot(624)
stairs(t, tau1(2, 1:size(t,2)),'r', 'linewidth', 1.2)
hold on
stairs(t, tau2(2, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, 600*ones(1,NoS),'-.g')
stairs(t, -600*ones(1,NoS),'-.g')
ylim([-650 650])
ylabel('$\tau_Y$ [N]', 'interpreter','latex')

subplot(626)
stairs(t, tau1(3, 1:size(t,2)),'r', 'linewidth', 1.2)
hold on
stairs(t, tau2(3, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, 600*ones(1,NoS),'-.g')
stairs(t, -600*ones(1,NoS),'-.g')
ylim([-650 650])
ylabel('$\tau_Z$ [N]', 'interpreter','latex')

subplot(628)
stairs(t, tau1(4, 1:size(t,2)),'r', 'linewidth', 1.2)
hold on
stairs(t, tau2(4, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, 300*ones(1,NoS),'-.g')
stairs(t, -300*ones(1,NoS),'-.g')
ylim([-350 350])
ylabel('$\tau_K$ [N]', 'interpreter','latex')

subplot(6,2,10)
stairs(t, tau1(5, 1:size(t,2)),'r', 'linewidth', 1.2)
hold on
stairs(t, tau2(5, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, 300*ones(1,NoS),'-.g')
stairs(t, -300*ones(1,NoS),'-.g')
ylim([-350 350])
ylabel('$\tau_M$ [N]', 'interpreter','latex')

subplot(6,2,12)
stairs(t, tau1(6, 1:size(t,2)),'r', 'linewidth', 1.2)
hold on
stairs(t, tau2(6, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, 300*ones(1,NoS),'-.g')
stairs(t, -300*ones(1,NoS),'-.g')
ylim([-350 350])
ylabel('$\tau_N$ [N]', 'interpreter','latex')
xlabel ('Time [s]', 'interpreter','latex')


%% Plots of 3D motion and position variables in the x, y and z directions
xlimit = [-16-1.5, 5+1.5];
ylimit = [-25-1.5, 5+1.5];
zlimit = [0, -12]; % Optional

figure;
subplot(221)
% Plot reference trajectory
plot3(yref(1, 1:size(t,2)), yref(2, 1:size(t,2)), yref(3, 1:size(t,2)), 'k:', 'LineWidth', 2);
hold on; grid on;
% Plot test trajectories
plot3(y1(1, 1:size(t,2)), y1(2, 1:size(t,2)), y1(3, 1:size(t,2)), 'b--', 'LineWidth', 2);
plot3(y2(1, 1:size(t,2)), y2(2, 1:size(t,2)), y2(3, 1:size(t,2)), 'r-', 'LineWidth', 2);
% view(3);
% Left wall (x = xlim(1))
fill3([xlimit(1) xlimit(1) xlimit(1) xlimit(1)], ...
      [ylimit(1) ylimit(2) ylimit(2) ylimit(1)], ...
      [zlimit(1) zlimit(1) zlimit(2) zlimit(2)], ...
      'c', 'FaceAlpha', 0.2);

% Right wall (x = xlim(2))
fill3([xlimit(2) xlimit(2) xlimit(2) xlimit(2)], ...
      [ylimit(1) ylimit(2) ylimit(2) ylimit(1)], ...
      [zlimit(1) zlimit(1) zlimit(2) zlimit(2)], ...
      'c', 'FaceAlpha', 0.2);

% Front wall (y = ylim(1))
fill3([xlimit(1) xlimit(2) xlimit(2) xlimit(1)], ...
      [ylimit(1) ylimit(1) ylimit(1) ylimit(1)], ...
      [zlimit(1) zlimit(1) zlimit(2) zlimit(2)], ...
      'c', 'FaceAlpha', 0.2);

% Back wall (y = ylim(2))
fill3([xlimit(1) xlimit(2) xlimit(2) xlimit(1)], ...
      [ylimit(2) ylimit(2) ylimit(2) ylimit(2)], ...
      [zlimit(1) zlimit(1) zlimit(2) zlimit(2)], ...
      'c', 'FaceAlpha', 0.2);
xlabel('$x(t)$ [m]', 'interpreter','latex')
ylabel('$y(t)$ [m]', 'interpreter','latex')
zlabel('$z(t)$ [m]', 'interpreter','latex')
legend('Reference', 'Test 1', 'Test 2','Workspace Barrier');



subplot(222)
stairs(t, x1(1, 1:size(t,2)),'-r', 'linewidth', 1.2)
hold on
stairs(t, x2(1, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, yref(1, 1:size(t,2)),'-.k', 'linewidth', 1.2)
ylabel('$x$ [m]', 'interpreter','latex')
xlabel('Time [s]')
ylim([-20 5])
grid on
axes('Position', [0.1 0.1 0.1 0.1]);
box on; grid on

subplot(223)
stairs(t, x1(2, 1:size(t,2)),'-r', 'linewidth', 1.2)
hold on
stairs(t, x2(2, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, yref(2, 1:size(t,2)),'-.k', 'linewidth', 1.2)
ylim([-35 5])
ylabel('$y$ [m]', 'interpreter','latex')
xlabel('Time [s]')
grid on

subplot(224)
stairs(t, x1(6, 1:size(t,2)),'-r', 'linewidth', 1.2)
hold on
stairs(t, x2(6, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, yref(6, 1:size(t,2)),'-.k', 'linewidth', 1.2)
ylim([-1 1])
ylabel('$z$ [m]', 'interpreter','latex')
xlabel('Time [s]')
grid on

%% plot 3D
figure;
% Plot reference trajectory
plot3(yref(1, 1:size(t,2)), yref(2, 1:size(t,2)), yref(3, 1:size(t,2)), 'k:', 'LineWidth', 2);
hold on; grid on;
% Plot test trajectories
plot3(y1(1, 1:size(t,2)), y1(2, 1:size(t,2)), y1(3, 1:size(t,2)), 'b--', 'LineWidth', 2);
plot3(y2(1, 1:size(t,2)), y2(2, 1:size(t,2)), y2(3, 1:size(t,2)), 'r-', 'LineWidth', 2);
% view(3);
% Left wall (x = xlim(1))
fill3([xlimit(1) xlimit(1) xlimit(1) xlimit(1)], ...
      [ylimit(1) ylimit(2) ylimit(2) ylimit(1)], ...
      [zlimit(1) zlimit(1) zlimit(2) zlimit(2)], ...
      'c', 'FaceAlpha', 0.2);

% Right wall (x = xlim(2))
fill3([xlimit(2) xlimit(2) xlimit(2) xlimit(2)], ...
      [ylimit(1) ylimit(2) ylimit(2) ylimit(1)], ...
      [zlimit(1) zlimit(1) zlimit(2) zlimit(2)], ...
      'c', 'FaceAlpha', 0.2);

% Front wall (y = ylim(1))
fill3([xlimit(1) xlimit(2) xlimit(2) xlimit(1)], ...
      [ylimit(1) ylimit(1) ylimit(1) ylimit(1)], ...
      [zlimit(1) zlimit(1) zlimit(2) zlimit(2)], ...
      'c', 'FaceAlpha', 0.2);

% Back wall (y = ylim(2))
fill3([xlimit(1) xlimit(2) xlimit(2) xlimit(1)], ...
      [ylimit(2) ylimit(2) ylimit(2) ylimit(2)], ...
      [zlimit(1) zlimit(1) zlimit(2) zlimit(2)], ...
      'c', 'FaceAlpha', 0.2);
xlabel('$x(t)$ [m]', 'interpreter','latex')
ylabel('$y(t)$ [m]', 'interpreter','latex')
zlabel('$z(t)$ [m]', 'interpreter','latex')
legend('Reference', 'Test 1', 'Test 2','Workspace Barrier');
