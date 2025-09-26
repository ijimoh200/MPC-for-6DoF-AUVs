% Plots for Dubins path tracking

figure
grid on
plot3(yref(1, :), yref(2, :), yref(3, :),'--k')
hold on
plot3(x1(1, :), x1(2, :), x1(3, :),'Linestyle','--','Color', "#7E2F8E",...
    'MarkerFaceColor','red',...
    'MarkerSize',15)
plot3(x2(1, :), x2(2, :), x2(3, :),'Linestyle','--','Color',	'r',...
    'MarkerFaceColor','red',...
    'MarkerSize',15)
plot3(x3(1, :), x3(2, :), x3(3, :),'Linestyle',':','Color', "#EDB120",...
    'MarkerFaceColor','red',...
    'MarkerSize',15)
plot3(x4(1, :), x4(2, :), x4(3, :),'Linestyle',':','Color', "#4DBEEE",...
    'MarkerFaceColor','red',...
    'MarkerSize',15)
% plot3(yrefcalc(1, :), yrefcalc(2, :), yrefcalc(3, :),'-.b')
grid on
xlabel('$x(t)$ [m]', 'interpreter','latex')
ylabel('$y(t)$ [m]', 'interpreter','latex')
zlabel('$z(t)$ [m]', 'interpreter','latex')
legend('Desired','TMPC ($R_a=0.1$)','TMPC ($R_a=0.2$)','TMPC ($R_a=0.4$)','NMPC', 'interpreter','latex','Orientation','horizontal')

% 
% figure
% plot(t,y1(3, 1:size(t,2)))
% hold on
% plot(t,yref(3, 1:size(t,2)))

% Output plots

figure
subplot(321)
stairs(t, yref(1, 1:size(t,2)),'--k', 'linewidth', 1.2)
hold on
stairs(t, (y1(1, 1:size(t,2))),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
stairs(t, (y2(1, 1:size(t,2))),'--r', 'linewidth', 1.2),
stairs(t, (y3(1, 1:size(t,2))),'Linestyle',':','Color', "#EDB120",'linewidth', 1.2)
stairs(t, (y4(1, 1:size(t,2))),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
% stairs(t,zeros(1,NoS),'-.k')
% legend('Proposed MPC','Zhang et al. (2019)')
legend('Desired','TMPC ($R_a=0.1$)','TMPC ($R_a=0.2$)','TMPC ($R_a=0.4$)','NMPC', 'interpreter','latex','Orientation','horizontal')
ylabel('${x}$ [m]', 'interpreter','latex')
% xlabel('Time [s]')
ylim([-5 22])
xlim([0 480])
grid on
axes('position',[.65 .652 .252 .252])
box on


subplot(323)
stairs(t, yref(2, 1:size(t,2)),'--k', 'linewidth', 1.2)
hold on
stairs(t, (y1(2, 1:size(t,2))),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
stairs(t, (y2(2, 1:size(t,2))),'--r', 'linewidth', 1.2),
stairs(t, (y3(2, 1:size(t,2))),'Linestyle',':','Color', "#EDB120",'linewidth', 1.2)
stairs(t, (y4(2, 1:size(t,2))),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
%stairs(t,zeros(1,NoS),'-.k')
ylabel('${y}$ [m]', 'interpreter','latex')
% xlabel ('Time [s]', 'interpreter','latex')
ylim([2 22])
xlim([0 480])
grid on

subplot(325)
% figure
stairs(t, yref(3, 1:size(t,2)),'--k', 'linewidth', 1.2)
hold on
stairs(t, 1*(y1(3, 1:size(t,2))),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
stairs(t, 1*(y2(3, 1:size(t,2))),'--r', 'linewidth', 1.2),
stairs(t, 1*(y3(3, 1:size(t,2))),'Linestyle',':','Color', "#EDB120",'linewidth', 1.2)
stairs(t, 1*(y4(3, 1:size(t,2))),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
%stairs(t,zeros(1,NoS),'-.k')
ylim([-28 0])
ylabel('${z}$ [m]', 'interpreter','latex')
xlabel ('Time [s]', 'interpreter','latex')
xlim([0 480])
grid on


subplot(322)
stairs(t, yref(4, 1:size(t,2)),'--k', 'linewidth', 1.2)
hold on
stairs(t, (y1(4, 1:size(t,2))),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
stairs(t, (y2(4, 1:size(t,2))),'--r', 'linewidth', 1.2),
stairs(t, (y3(4, 1:size(t,2))),'Linestyle',':','Color', "#EDB120",'linewidth', 1.2)
stairs(t, (y4(4, 1:size(t,2))),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
%stairs(t,zeros(1,NoS),'-.k')
% legend('Proposed MPC','Zhang et al. (2019)')
ylabel('${\phi}$ [rad]', 'interpreter','latex')
% xlabel('Time [s]')
ylim([-0.2 0.2])
xlim([0 480])
grid on

subplot(324)
% figure
stairs(t, yref(5, 1:size(t,2)),'--k', 'linewidth', 1.2)
hold on
stairs(t, (y1(5, 1:size(t,2))),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
stairs(t, (y2(5, 1:size(t,2))),'--r', 'linewidth', 1.2),
stairs(t, (y3(5, 1:size(t,2))),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
stairs(t, (y4(5, 1:size(t,2))),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
%stairs(t,zeros(1,NoS),'-.k')
ylabel('${\theta}$ [rad]', 'interpreter','latex')
% xlabel ('Time [s]', 'interpreter','latex')
ylim([-0.1 0.1])
xlim([0 480])
grid on

subplot(326)
stairs(t, yref(6, 1:size(t,2)),'--k', 'linewidth', 1.2)
hold on
stairs(t, (y1(6, 1:size(t,2))),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
stairs(t, (y2(6, 1:size(t,2))),'--r', 'linewidth', 1.2)
stairs(t, (y3(6, 1:size(t,2))),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
stairs(t, (y4(6, 1:size(t,2))),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
%stairs(t,zeros(1,NoS),'-.k')
ylim([-0.5 4])
ylabel('${\psi}$ [rad]', 'interpreter','latex')
xlabel ('Time [s]', 'interpreter','latex')
xlim([0 480])
grid on


% velocities plots

figure
subplot(321)
stairs(t, x1(7, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
hold on
stairs(t, x2(7, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, x3(7, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
stairs(t, x4(7, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
ylabel('$u$ [m/s]', 'interpreter','latex')
% xlabel('Time [s]')
ylim([-0.6 0.25])
grid on
xlim([0 480])
legend('TMPC ($R_a=0.1$)','TMPC ($R_a=0.2$)','TMPC ($R_a=0.4$)','NMPC', 'interpreter','latex','Orientation','horizontal')
axes('position',[.65 .652 .252 .252])
box on

subplot(323)
% figure
stairs(t, x3(8, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
hold on
stairs(t, x4(8, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
stairs(t, x2(8, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, x1(8, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
ylabel('$v$ [m/s]', 'interpreter','latex')
% xlabel('Time [s]')
ylim([-1 0.2])
xlim([0 480])
grid on


subplot(325)
stairs(t, x3(9, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
hold on
stairs(t, x4(9, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
stairs(t, x2(9, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, x1(9, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
ylabel('$w$ [m/s]', 'interpreter','latex')
xlabel('Time [s]')
ylim([-0.3 0.1])
xlim([0 480])
grid on


subplot(322)
% figure
stairs(t, x3(10, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
hold on
stairs(t, x4(10, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
stairs(t, x2(10, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, x1(10, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
ylabel('$p$ [rad/s]', 'interpreter','latex')
% xlabel('Time [s]')
ylim([-1.5 1.5])
xlim([0 480])
grid on



subplot(324)
% figure
stairs(t, x3(11, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
hold on
stairs(t, x4(11, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
stairs(t, x2(11, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, x1(11, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
ylabel('$q$ [rad/s]', 'interpreter','latex')
% xlabel('Time [s]')
ylim([-0.05 0.05])
xlim([0 480])
grid on




subplot(326)
% figure
stairs(t, x3(12, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
hold on
stairs(t, x4(12, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
stairs(t, x2(12, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, x1(12, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
ylabel('$r$ [rad/s]', 'interpreter','latex')
xlabel('Time [s]')
% legend('$R_a=0.4$','$R_a=0.6$','$R_a=0.8$', 'interpreter','latex','Orientation','horizontal')
ylim([-0.2 0.2])
xlim([0 480])
grid on


% input plots

figure
subplot(621)
stairs(t, tau3(1, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
hold on
stairs(t, tau4(1, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
stairs(t, tau2(1, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, tau1(1, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
stairs(t, 1000*ones(1,NoS),'Color',	"#A2142F")
stairs(t, -1000*ones(1,NoS),'Color',	"#A2142F")
ylim([-1100 1100])
ylabel('$\tau_X$ [N]', 'interpreter','latex')
xlabel ('Time [s]', 'interpreter','latex')
grid on
xlim([0 480])
axes('position',[.65 .652 .252 .252])
box on



tau4_sat =  max(min(tau4(2, 1:size(t,2)), 1000), -1000);
subplot(623)
% figure
stairs(t, tau3(2, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
hold on
stairs(t, tau4(2, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
stairs(t, tau2(2, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, tau1(2, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
stairs(t, 1000*ones(1,NoS),'Color',	"#A2142F")
stairs(t, -1000*ones(1,NoS),'Color',	"#A2142F")
ylim([-1100 1100])
% legend('$R_a=0.4$','$R_a=0.6$','$R_a=0.8$', 'interpreter','latex','Orientation','horizontal')
ylabel('$\tau_Y$ [N]', 'interpreter','latex')
xlabel ('Time [s]', 'interpreter','latex')
grid on
xlim([0 480])


subplot(625)
% figure
stairs(t, tau1(3, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
hold on
stairs(t, tau2(3, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, tau3(3, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
stairs(t, tau4(3, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
stairs(t, 1000*ones(1,NoS),'Color',	"#A2142F")
stairs(t, -1000*ones(1,NoS),'Color',	"#A2142F")
ylim([-1100 1100])
legend('TMPC ($R_a=0.1$)','TMPC ($R_a=0.2$)','TMPC ($R_a=0.4$)','NMPC', 'interpreter','latex','Orientation','horizontal')
ylabel('$\tau_Z$ [N]', 'interpreter','latex')
xlabel ('Time [s]', 'interpreter','latex')
grid on
xlim([0 480])

subplot(627)
% figure
stairs(t, tau3(4, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
hold on
stairs(t, tau4(4, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
stairs(t, tau2(4, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, tau1(4, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
stairs(t, 1000*ones(1,NoS),'Color',	"#A2142F")
stairs(t, -1000*ones(1,NoS),'Color',	"#A2142F")
ylim([-1100 1100])
% legend('$R_a=0.4$','$R_a=0.6$','$R_a=0.8$', 'interpreter','latex','Orientation','horizontal')
ylabel('$\tau_K$ [Nm]', 'interpreter','latex')
xlabel ('Time [s]', 'interpreter','latex')
grid on
xlim([0 480])


subplot(629)
% figure
stairs(t, tau3(1, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
hold on
stairs(t, tau4(1, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
stairs(t, tau2(1, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, tau1(1, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
stairs(t, 1000*ones(1,NoS),'Color',	"#A2142F")
stairs(t, -1000*ones(1,NoS),'Color',	"#A2142F")
ylim([-1100 1100])
% legend('$R_a=0.4$','$R_a=0.6$','$R_a=0.8$', 'interpreter','latex','Orientation','horizontal')
ylabel('$\tau_M$ [Nm]', 'interpreter','latex')
xlabel ('Time [s]', 'interpreter','latex')
grid on
xlim([0 480])

subplot(6,2,11)
% figure
stairs(t, tau3(1, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
hold on
stairs(t, tau4(1, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
stairs(t, tau2(1, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, tau1(1, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
stairs(t, 1000*ones(1,NoS),'Color',	"#A2142F")
stairs(t, -1000*ones(1,NoS),'Color',	"#A2142F")
ylim([-1100 1100])
% legend('$R_a=0.4$','$R_a=0.6$','$R_a=0.8$', 'interpreter','latex','Orientation','horizontal')
ylabel('$\tau_N$ [Nm]', 'interpreter','latex')
xlabel ('Time [s]', 'interpreter','latex')
grid on
xlim([0 480])

% input rates plots

subplot(622)
% figure
stairs(t, du3(1, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
hold on
stairs(t, du4(1, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
stairs(t, du2(1, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, du1(1, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
stairs(t, 100*ones(1,NoS),'Color',	"#A2142F")
stairs(t, -100*ones(1,NoS),'Color',	"#A2142F")
ylim([-110 110])
% legend('$\tau_X$,','$\tau_Y$','$\tau_Z$', 'interpreter','latex')
ylabel('$\delta\tau_X$ [N]', 'interpreter','latex')
xlabel ('Time [s]', 'interpreter','latex')
grid on
% xlim([0 120])
xlim([0 480])

subplot(624)
stairs(t, du3(2, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
hold on
stairs(t, du4(2, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
stairs(t, du2(2, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, du1(2, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
stairs(t, 100*ones(1,NoS),'Color',	"#A2142F")
stairs(t, -100*ones(1,NoS),'Color',	"#A2142F")
ylim([-110 110])
% legend('$\tau_K$,','$\tau_M$','$\tau_N$', 'interpreter','latex')
ylabel('$\delta\tau_Y$ [N]', 'interpreter','latex')
xlabel ('Time [s]', 'interpreter','latex')
grid on
xlim([0 480])


subplot(626)
% figure
stairs(t, du3(3, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
hold on
stairs(t, du4(3, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
stairs(t, du2(3, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, du1(3, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
stairs(t, 100*ones(1,NoS),'Color',	"#A2142F")
stairs(t, -100*ones(1,NoS),'Color',	"#A2142F")
ylim([-110 110])
% legend('$\delta_Z$,','$\tau_M$','$\tau_N$', 'interpreter','latex')
ylabel('$\delta\tau_Z$ [N]', 'interpreter','latex')
xlabel ('Time [s]', 'interpreter','latex')
grid on
xlim([0 480])

subplot(628)
% figure
stairs(t, du3(4, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
hold on
stairs(t, du4(4, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
stairs(t, du2(4, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, du1(4, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
stairs(t, 100*ones(1,NoS),'Color',	"#A2142F")
stairs(t, -100*ones(1,NoS),'Color',	"#A2142F")
ylim([-110 110])
% legend('$\delta_Z$,','$\tau_M$','$\tau_N$', 'interpreter','latex')
ylabel('$\delta\tau_K$ [Nm]', 'interpreter','latex')
xlabel ('Time [s]', 'interpreter','latex')
grid on
xlim([0 480])


subplot(6,2,10)
stairs(t, du3(5, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
hold on
stairs(t, du4(5, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
stairs(t, du2(5, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, du1(5, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
stairs(t, 100*ones(1,NoS),'Color',	"#A2142F")
stairs(t, -100*ones(1,NoS),'Color',	"#A2142F")
ylim([-110 110])
% legend('$\delta_Z$,','$\tau_M$','$\tau_N$', 'interpreter','latex')
ylabel('$\delta\tau_M$ [Nm]', 'interpreter','latex')
xlabel ('Time [s]', 'interpreter','latex')
grid on
xlim([0 480])


subplot(6,2,12)
% figure
stairs(t, du3(6, 1:size(t,2)),'Linestyle',':','Color', "#EDB120", 'linewidth', 1.2)
hold on
stairs(t, du4(6, 1:size(t,2)),'Linestyle',':','Color', "#4DBEEE", 'linewidth', 1.2)
stairs(t, du2(6, 1:size(t,2)),'--r', 'linewidth', 1.2)
stairs(t, du1(6, 1:size(t,2)),'Linestyle',':','Color', "#7E2F8E", 'linewidth', 1.2)
stairs(t, 100*ones(1,NoS),'Color',	"#A2142F")
stairs(t, -100*ones(1,NoS),'Color',	"#A2142F")
ylim([-110 110])
% legend('$\delta_Z$,','$\tau_M$','$\tau_N$', 'interpreter','latex')
ylabel('$\delta\tau_N$ [Nm]', 'interpreter','latex')
xlabel ('Time [s]', 'interpreter','latex')
grid on
xlim([0 480])