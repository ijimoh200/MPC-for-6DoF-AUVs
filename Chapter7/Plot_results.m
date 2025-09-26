%% Plot results 
t = (0:tfinal-Ts)/10;
% Wave disturbance
figure
plot(t, tau_wave(1, 1:tfinal))
ylabel('Wave Force [N]')
xlabel('Time [s]')
grid on
xlim([0 tfinal/10])

% plot computational time
figure
subplot(211)
plot(ctime(1,1:tfinal)*1000)
subplot(212)
plot(ctime(2,1:tfinal)*1000)


% First obstacle position
xo = Poq(1,1); yo = Poq(2,1); zo = Poq(3,1);
centre = [xo,yo];
% Second obstacle position
xo2 = Poq(1,1); yo2 = Poq(2,1); zo2 = Poq(3,1);
radius = 4;
centre2 = [xo2,yo2];
[xs,ys,zs] = sphere;
xs = xs*radius;
ys = ys*radius;
zs = zs*radius;

% 2D plot
figure
plot(x1(1,1:tfinal),x1(2,1:tfinal),'-r','LineWidth',1.5)
hold on
plot(1000*yref(1,:),1000*yref(2,:),'-g')
plot(yref(1,:),yref(2,:),'--k')
plot(yref(1,:),yref(2,:),'ob')
plot(1000*yref(1,:),1000*yref(2,:),'or')
viscircles( centre,radius,'Color','g')
viscircles( centre2,radius,'Color','g')
legend('AUV Path','Safe region','Reference','Waypoint')
xlabel('$x$ [m]','Interpreter','latex')
ylabel('$y$ [m]','Interpreter','latex')
xlim([-5 75])
ylim([-5 75])
grid on

% 3D plot
figure
plot3(x1(1, 1:tfinal-1), x1(2, 1:tfinal-1), x1(3, 1:tfinal-1),'-r',...
    'MarkerFaceColor','red',...
    'MarkerSize',15)
hold on
plot3(yref(1, :), yref(2, :), yref(3, :),'ob')
plot3(yref(1, :), yref(2, :), yref(3, :),'--k')
grid on
xlabel('$x(t)$ [m]', 'interpreter','latex')
ylabel('$y(t)$ [m]', 'interpreter','latex')
zlabel('$z(t)$ [m]', 'interpreter','latex')
legend('AUV path (GS1)','Waypoint','Reference')
surf(xs+xo2,ys+yo2,zs+zo2,'FaceColor', "#77AC30")
surf(xs+xo,ys+yo,zs+zo,'FaceColor', "#77AC30")
grid on
xlabel('$x(t)$ [m]', 'interpreter','latex')
ylabel('$y(t)$ [m]', 'interpreter','latex')
zlabel('$z(t)$ [m]', 'interpreter','latex')
legend('AUV path','Waypoint','Reference')
grid on


% plot of velocities and angular positions
length = tfinal*Ts;
figure
subplot(211)
plot(t,x1(7, 1:tfinal),'-r','LineWidth',1.5)
hold on
plot(t,x1(8, 1:tfinal),'--b','LineWidth',1.5)
plot(t,x1(9, 1:tfinal),'-.k','LineWidth',1.5)
legend('$u$','$v$','$w$','Interpreter','latex')
ylim([-2 2])
xlim([0 length])
ylabel('Linear velocities [m/s]')
xlabel('Time [s]')
grid on

subplot(212)
plot(t,x1(4, 1:tfinal)*180/pi,'-r','LineWidth',1.5)
hold on
plot(t,x1(5, 1:tfinal)*180/pi,'--b','LineWidth',1.5)
plot(t,x1(6, 1:tfinal)*180/pi,'-.k','LineWidth',1.5)
legend('$\phi$','$\theta$','$\psi$','Interpreter','latex')
ylim([-120 200])
xlim([0 length])
ylabel('Angular position [$\deg /s$]','Interpreter','latex')
xlabel('Time [s]')
grid on

% Input forces and moments
figure
subplot(211)
plot(t,u1(1, 1:tfinal),'-r','LineWidth',1.5)
hold on
plot(t,u1(2, 1:tfinal),'--b','LineWidth',1.5)
plot(t,u1(3, 1:tfinal),'-.k','LineWidth',1.5)
plot(t,2000*ones(1, tfinal),':k','LineWidth',1.5)
plot(t,-2000*ones(1, tfinal),':k','LineWidth',1.5)
ylim([-2200 2200])
xlim([0 length])
ylabel('Input forces [N]')
legend('$\tau_u$','$\tau_v$','$\tau_w$','Interpreter','latex')
grid on

subplot(212)
plot(t,u1(4, 1:tfinal),'-r','LineWidth',1.5)
hold on
plot(t,u1(5, 1:tfinal),'--b','LineWidth',1.5)
plot(t,u1(6, 1:tfinal),'-.k','LineWidth',1.5)
plot(t,2000*ones(1, tfinal),':k','LineWidth',1.5)
plot(t,-2000*ones(1, tfinal),':k','LineWidth',1.5)
ylim([-2200 2200])
xlim([0 length])
ylabel('Input moments [Nm]')
xlabel('Time [s]')
legend('$\tau_p$','$\tau_q$','$\tau_r$','Interpreter','latex')
grid on


%% Organise and plot data regarding computational time
% Data from the table
N = [12, 14, 16, 18];  % Values of N

% Building and Solver Times for each Nu value
BuildingTime_Nu1 = [14.71, 17.05, 23.52, 27.08];
SolverTime_Nu1 = [44.36, 47.74, 57.33, 66.29];

BuildingTime_Nu3 = [16.75, 20.18, 25.15, 29.64];
SolverTime_Nu3 = [53.52, 59.52, 64.79, 71.43];

BuildingTime_Nu6 = [18.26, 22.13, 26.43, 31.44];
SolverTime_Nu6 = [57.19, 66.07, 74.41, 81.1];


BuildingTime_MM = [14.26*ones(1,4)];
SolverTime_MM = [826*ones(1,4)];

% Create figure
figure;
hold on;

% Plot Building Times
plot(N, BuildingTime_Nu1, '-o','LineWidth', 1.5);
plot(N, BuildingTime_Nu3, '-s','LineWidth', 1.5);
plot(N, BuildingTime_Nu6, '-^', 'LineWidth', 1.5);

% Plot Solver Times
plot(N, SolverTime_Nu1, '--o', 'LineWidth', 1.5);
plot(N, SolverTime_Nu3, '--s','LineWidth', 1.5);
plot(N, SolverTime_Nu6, '--^', 'LineWidth', 1.5);
plot(N, BuildingTime_MM, '-b', 'LineWidth', 1.5);
plot(N, SolverTime_MM, '--b',  'LineWidth', 1.5);

% Add labels and title
xlabel('Prediction horizon, $N$', 'Interpreter', 'latex');
ylabel('Time [ms]');

% Change y-axis to logarithmic scale
set(gca, 'YScale', 'log');

% Display legend with LaTeX interpreter
legend('Building Time, $N_u=1$', 'Building Time, $N_u=3$', 'Building Time, $N_u=6$', ...
    'Solver Time, $N_u=1$', 'Solver Time, $N_u=3$', 'Solver Time, $N_u=6$', 'Location', 'northwest', 'Interpreter', 'latex');
grid on;
hold off;

