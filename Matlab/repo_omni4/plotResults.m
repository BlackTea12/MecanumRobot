% PLOT RESULTS
close all; clc;
%% trajectory
figure(1);
time = [1:length(out.tout)];
robot_X = getdatasamples(out.robot_pos_x, time);
robot_Y = getdatasamples(out.robot_pos_y, time);
plot(0,0,'ro','LineWidth',2);
grid on; hold on;
xlim([-0.1 1.3]); ylim([-0.1 1.3]);
for i = 1:length(robot_X)
ppp = plot(robot_X(i), robot_Y(i),'go','LineWidth',1.5); % grid on; hold on;
drawnow;
ppp(delete);
%plot(0,0,'ro','LineWidth',2);
end
% plot(robot_X, robot_Y,'g--','LineWidth',1.5); grid on; hold on;
% plot(0,0,'ro','LineWidth',2);
% xlim([-1.4 0.1]); ylim([-0.5 0.5]);
xlabel('X[m]'); ylabel('Y[m]');
title('Trajectory of 4 mecanum wheeled robot','fontsize',14);
hold off;

%% vehicle speed according to x and y
figure(2);
time = [1:length(out.tout)];
robot_vel = getdatasamples(out.robot_vel, time);
plot(out.tout, robot_vel,'LineWidth',1.5); grid on; hold on;
%xlim([-1.4 0.1]); ylim([-0.5 0.5]);
xlabel('time[sec]'); ylabel('velocity[m/s]');
title('Speed f 4 mecanum wheeled robot','fontsize',14);
hold off;

%% yaw angle and yaw rate
figure(3);
time = [1:length(out.tout)];
yaw = getdatasamples(out.robot_yaw, time);
yawrate = getdatasamples(out.robot_yawrate, time);

subplot(2,1,1); 
xlabel('time[sec]'); ylabel('yaw[deg]');
%title('Yaw Angle when rotating at 30 deg/sec wheel speed','fontsize',14); hold on; grid on;
title('Yaw Angle','fontsize',14); hold on; grid on;
plot(out.tout, rad2deg(yaw),'r-.','LineWidth',1.5); 
%xlim([-1.4 0.1]); ylim([-0.5 0.5]);
hold off;

subplot(2,1,2); 
xlabel('time[sec]'); ylabel('yaw rate[deg/sec]');
%title('Yaw Rate when rotating at 30 deg/sec wheel speed','fontsize',14); hold on; grid on;
title('Yaw Rate','fontsize',14); hold on; grid on;
plot(out.tout, rad2deg(yawrate),'r-.','LineWidth',1.5); 
%xlim([-1.4 0.1]); ylim([-0.5 0.5]);
hold off;