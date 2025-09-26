% Plot of wave disturbance
figure
plot(t, tau_wave(1,1:size(t,2)))
ylabel('$\tau^w_i (i=X,Y,Z,K,M,N)$',Interpreter='latex')
xlabel('Time [s]',Interpreter='latex')
title('Ocean Wave Disturbances')

% Plots showing the convergence of the AUV acceleration i.e., velocity
% change at the end of the prediction horizon
RXpred1 = reshape(dXpred(:,3410),[12,N]);
RXpred2 = reshape(dXpred(:,3430),[12,N]);
RXpred3 = reshape(dXpred(:,3550),[12,N]);
RXpred4 = reshape(dXpred(:,3570),[12,N]);
RXpred5 = reshape(dXpred(:,3590),[12,N]);

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
legend('$k=3410$','$k=3430$','$k=3450$','$k=3470$','$k=3490$','Orientation','horizontal',Interpreter='latex')
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
ylabel('$\Delta w$ [m/s]',Interpreter='latex')
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


% Plots of 3D motion and position variables in the x, y and z directions
figure
subplot(221)
grid on
simstart = find(x1(3, 1:size(t,2)) == x1(3, 1));
simend = find(x1(3, 1:size(t,2)) == x1(3, end));
plot3(y1(1, 1:size(t,2)), y1(2, 1:size(t,2)), y1(3, 1:size(t,2)),'-r','MarkerIndices',[simstart simend],...
    'MarkerFaceColor','red',...
    'MarkerSize',15)
hold on
plot3(x2(1, 1:size(t,2)), x2(2, 1:size(t,2)), x2(3, 1:size(t,2)),'--b')
plot3(yref(1, 1:size(t,2)), yref(2, 1:size(t,2)), yref(3, 1:size(t,2)),'-.k')
grid on
xlabel('$x(t)$ [m]', 'interpreter','latex')
ylabel('$y(t)$ [m]', 'interpreter','latex')
zlabel('$z(t)$ [m]', 'interpreter','latex')
legend('MPC4 (LPVMPC2)','MPC2','Reference Path')

txt1 = '\leftarrow AUV start point';
text(2,8,txt1)
txt2 = '\leftarrow AUV dock position';
text(-8.5, -7.6,-17.5,txt2)


subplot(222)
stairs(t, x1(1, 1:size(t,2)),'-r', 'linewidth', 1.2)
hold on
stairs(t, x2(1, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, yref(1, 1:size(t,2)),'-.k', 'linewidth', 1.2)
ylabel('$x$ [m]', 'interpreter','latex')
xlabel('Time [s]')
ylim([-12 12])
grid on
axes('Position', [0.1 0.1 0.1 0.1]);
box on; grid on

subplot(223)
stairs(t, x1(2, 1:size(t,2)),'-r', 'linewidth', 1.2)
hold on
stairs(t, x2(2, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, yref(2, 1:size(t,2)),'-.k', 'linewidth', 1.2)
ylabel('$y$ [m]', 'interpreter','latex')
xlabel('Time [s]')
grid on

subplot(224)
stairs(t, x1(3, 1:size(t,2)),'-r', 'linewidth', 1.2)
hold on
stairs(t, x2(3, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, yref(3, 1:size(t,2)),'-.k', 'linewidth', 1.2)
ylabel('$z$ [m]', 'interpreter','latex')
xlabel('Time [s]')
grid on



% Plots of velocity variables and input forces and moments
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




% 
figure
subplot(121)
plot(t,e_l1,'r', 'linewidth', 1.2)
hold on
plot(t,e_l2,'--b', 'linewidth', 1.2)
plot(t,zeros(1,NoS),'-.k', 'linewidth', 1.2)
xlim([0 500])
xlabel ('Time [s]', 'interpreter','latex')
ylabel('$e_l$ [m]', 'interpreter','latex')
legend('Proposed MPC','Zhang et al., 2019')

subplot(122)
plot(t,e_a1,'r', 'linewidth', 1.2)
hold on
plot(t,e_a2,'--b', 'linewidth', 1.2)
plot(t,zeros(1,NoS),'-.k', 'linewidth', 1.2)
plot(t,zeros(1,NoS))
xlim([0 500])
xlabel ('Time [s]', 'interpreter','latex')
ylabel('$e_a$ [rad]', 'interpreter','latex')

