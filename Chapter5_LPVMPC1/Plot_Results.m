t = 0:Ts:Tf-Ts;
figure
subplot(621)
stairs(t, yref(1, :),'-.k', 'linewidth', 1.2)
hold on
stairs(t, y1(1, 1:size(t,2)),'-r', 'linewidth', 1.2)
stairs(t, y2(1, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, y3(1, 1:size(t,2)),':m', 'linewidth', 1.5)
ylabel('$x$ [m]', 'interpreter','latex')
grid on



tau1(:,end) = tau1(:,end-1);
tau2(:,end) = tau2(:,end-1);
subplot(623)
stairs(t, yref(2, 1:size(t,2)),'-.k', 'linewidth', 1.2)
hold on
stairs(t, y1(2, 1:size(t,2)),'r', 'linewidth', 1.2)
stairs(t, y2(2, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, y3(2, 1:size(t,2)),':m', 'linewidth', 1.5)
ylabel('$y$ [m]', 'interpreter','latex')
grid on


subplot(625)
stairs(t, yref(3, 1:size(t,2)),'-.k', 'linewidth', 1.2)
hold on
stairs(t, y1(3, 1:size(t,2)),'r', 'linewidth', 1.2)
stairs(t, y2(3, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, y3(3, 1:size(t,2)),':m', 'linewidth', 1.5)
%ylim([-0.05 0.03])
ylabel('$z$ [m]', 'interpreter','latex')
grid on


subplot(627)
stairs(t, yref(4, 1:size(t,2)),'-.k', 'linewidth', 1.2)
hold on
stairs(t, y1(4, 1:size(t,2)),'r', 'linewidth', 1.2)
stairs(t, y2(4, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, y3(4, 1:size(t,2)),':m', 'linewidth', 1.5)
ylabel('$\phi$ [rad]', 'interpreter','latex')
%ylim([-0.006 0.002])
grid on

subplot(629)
stairs(t, yref(5, 1:size(t,2)),'-.k', 'linewidth', 1.2)
hold on
stairs(t, y1(5, 1:size(t,2)),'r', 'linewidth', 1.2)
stairs(t, y2(5, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, y3(5, 1:size(t,2)),':m', 'linewidth', 1.5)
ylabel('$\theta$ [rad]', 'interpreter','latex')
grid on


subplot(6,2,11)
stairs(t, yref(6, 1:size(t,2)),'-.k', 'linewidth', 1.2)
hold on
stairs(t, y1(6, 1:size(t,2)),'r', 'linewidth', 1.2)
stairs(t, y2(6, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, y3(6, 1:size(t,2)),':m', 'linewidth', 1.5)
ylabel('$\psi$ [rad]', 'interpreter','latex')
legend('Target','MPC1', 'MPC2','MPC3')
xlabel ('Time [s]', 'interpreter','latex')
grid on


subplot(622)
stairs(t, tau1(1, 1:size(t,2)),'r', 'linewidth', 1.2)
hold on
stairs(t, tau2(1, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, tau3(1, 1:size(t,2)),':m', 'linewidth', 1.5)
ylabel('$\tau_X$ [N]', 'interpreter','latex')
grid on


subplot(624)
stairs(t, tau1(2, 1:size(t,2)),'r', 'linewidth', 1.2)
hold on
stairs(t, tau2(2, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, tau3(2, 1:size(t,2)),':m', 'linewidth', 1.5)
ylabel('$\tau_Y$ [N]', 'interpreter','latex')
grid on


subplot(626)
stairs(t, tau1(3, 1:size(t,2)),'r', 'linewidth', 1.2)
hold on
stairs(t, tau2(3, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, tau3(3, 1:size(t,2)),':m', 'linewidth', 1.5)
ylabel('$\tau_Z$ [N]', 'interpreter','latex')
grid on

subplot(628)
stairs(t, tau1(4, 1:size(t,2)),'r', 'linewidth', 1.2)
hold on
stairs(t, tau2(4, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, tau3(4, 1:size(t,2)),':m', 'linewidth', 1.5)
ylabel('$\tau_K$ [Nm]', 'interpreter','latex')
grid on



subplot(6,2,10)
stairs(t, tau1(5, 1:size(t,2)),'r', 'linewidth', 1.2)
hold on
stairs(t, tau2(5, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, tau3(5, 1:size(t,2)),':m', 'linewidth', 1.5)
ylabel('$\tau_M$ [Nm]', 'interpreter','latex')
grid on


subplot(6,2,12)
stairs(t, tau1(6, 1:size(t,2)),'r', 'linewidth', 1.2)
hold on
stairs(t, tau2(6, 1:size(t,2)),'--b', 'linewidth', 1.2)
stairs(t, tau3(6, 1:size(t,2)),':m', 'linewidth', 1.5)
xlabel ('Time [s]', 'interpreter','latex')
ylabel('$\tau_N$ [Nm]', 'interpreter','latex')
grid on


%% compare the proposed LPVMPC1 and LPVMPC2
% figure
% subplot(621)
% stairs(t, yref(1, :),'-.k', 'linewidth', 1.2)
% hold on
% stairs(t, y1(1, 1:size(t,2)),'-r', 'linewidth', 1.2)
% stairs(t, y4(1, 1:size(t,2)),'--b', 'linewidth', 1.2)
% ylabel('$x$ [m]', 'interpreter','latex')
% grid on
% 
% 
% 
% tau1(:,end) = tau1(:,end-1);
% tau4(:,end) = tau4(:,end-1);
% subplot(623)
% stairs(t, yref(2, 1:size(t,2)),'-.k', 'linewidth', 1.2)
% hold on
% stairs(t, y1(2, 1:size(t,2)),'r', 'linewidth', 1.2)
% stairs(t, y4(2, 1:size(t,2)),'--b', 'linewidth', 1.2)
% ylabel('$y$ [m]', 'interpreter','latex')
% grid on
% 
% 
% subplot(625)
% stairs(t, yref(3, 1:size(t,2)),'-.k', 'linewidth', 1.2)
% hold on
% stairs(t, y1(3, 1:size(t,2)),'r', 'linewidth', 1.2)
% stairs(t, y4(3, 1:size(t,2)),'--b', 'linewidth', 1.2)
% %ylim([-0.05 0.03])
% ylabel('$z$ [m]', 'interpreter','latex')
% grid on
% 
% 
% subplot(627)
% stairs(t, yref(4, 1:size(t,2)),'-.k', 'linewidth', 1.2)
% hold on
% stairs(t, y1(4, 1:size(t,2)),'r', 'linewidth', 1.2)
% stairs(t, y4(4, 1:size(t,2)),'--b', 'linewidth', 1.2)
% ylabel('$\phi$ [rad]', 'interpreter','latex')
% %ylim([-0.006 0.002])
% grid on
% 
% subplot(629)
% stairs(t, yref(5, 1:size(t,2)),'-.k', 'linewidth', 1.2)
% hold on
% stairs(t, y1(5, 1:size(t,2)),'r', 'linewidth', 1.2)
% stairs(t, y4(5, 1:size(t,2)),'--b', 'linewidth', 1.2)
% ylabel('$\theta$ [rad]', 'interpreter','latex')
% grid on
% 
% 
% subplot(6,2,11)
% stairs(t, yref(6, 1:size(t,2)),'-.k', 'linewidth', 1.2)
% hold on
% stairs(t, y1(6, 1:size(t,2)),'r', 'linewidth', 1.2)
% stairs(t, y4(6, 1:size(t,2)),'--b', 'linewidth', 1.2)
% ylabel('$\psi$ [rad]', 'interpreter','latex')
% legend('Target','MPC1', 'MPC2','MPC3')
% xlabel ('Time [s]', 'interpreter','latex')
% grid on
% inset_axes = axes('Position',[0.6 0.6 0.25 0.25]);
% box on
% grid on
% 
% 
% subplot(622)
% stairs(t, tau1(1, 1:size(t,2)),'r', 'linewidth', 1.2)
% hold on
% stairs(t, tau4(1, 1:size(t,2)),'--b', 'linewidth', 1.2)
% ylabel('$\tau_X$ [N]', 'interpreter','latex')
% grid on
% 
% 
% subplot(624)
% stairs(t, tau1(2, 1:size(t,2)),'r', 'linewidth', 1.2)
% hold on
% stairs(t, tau4(2, 1:size(t,2)),'--b', 'linewidth', 1.2)
% ylabel('$\tau_Y$ [N]', 'interpreter','latex')
% grid on
% 
% 
% subplot(626)
% stairs(t, tau1(3, 1:size(t,2)),'r', 'linewidth', 1.2)
% hold on
% stairs(t, tau4(3, 1:size(t,2)),'--b', 'linewidth', 1.2)
% ylabel('$\tau_Z$ [N]', 'interpreter','latex')
% grid on
% 
% subplot(628)
% stairs(t, tau1(4, 1:size(t,2)),'r', 'linewidth', 1.2)
% hold on
% stairs(t, tau4(4, 1:size(t,2)),'--b', 'linewidth', 1.2)
% ylabel('$\tau_K$ [Nm]', 'interpreter','latex')
% grid on
% 
% 
% 
% subplot(6,2,10)
% stairs(t, tau1(5, 1:size(t,2)),'r', 'linewidth', 1.2)
% hold on
% stairs(t, tau4(5, 1:size(t,2)),'--b', 'linewidth', 1.2)
% ylabel('$\tau_M$ [Nm]', 'interpreter','latex')
% grid on
% 
% 
% subplot(6,2,12)
% stairs(t, tau1(6, 1:size(t,2)),'r', 'linewidth', 1.2)
% hold on
% stairs(t, tau4(6, 1:size(t,2)),'--b', 'linewidth', 1.2)
% xlabel ('Time [s]', 'interpreter','latex')
% ylabel('$\tau_N$ [Nm]', 'interpreter','latex')
% grid on
% 
% % Error vectors
% error1 = y1 - yref;
% error2 = y4 - yref;
% 
% % Preallocate result vectors
% rmse1 = zeros(6,1); rmse_tau1 = zeros(6,1);
% rmse4 = zeros(6,1); rmse_tau4 = zeros(6,1);
% 
% % Loop through each row (state variable)
% for i = 1:6
%     % Controller 1
%     err1 = error1(i,:); %
%     rmse1(i,1) = sqrt(mean(err1.^2));
%     rmse_tau1(i,1) = rms(tau1(i,:));
% 
%     % Controller 2
%     err2 = error2(i,:);
%     rmse4(i,1) = sqrt(mean(err2.^2));
%     rmse_tau4(i,1) = rms(tau4(i,:));
% 
% end

