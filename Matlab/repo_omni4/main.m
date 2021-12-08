clear all; clc; close all; cla;
% set map
map = drawMapOccupancy('factory02.pgm', 10);

% get planning result
start = [12 1 0];
goal = [12 24 0];

tstart = tic;
[pthObj,solnInfo] = RRTStarPlannerSum(map, start, goal, 3, 10000, true);

tend = toc(tstart);
disp(tend);

% write for smoothing of path

t = 0:pthObj.NumStates-1;
tq = 0:0.01:pthObj.NumStates-1;
xtemp = interp1(t,pthObj.States(:,1),tq,'spline');
ytemp = interp1(t,pthObj.States(:,2),tq,'spline');

smtObj.States(1:size(xtemp,2),1) = transpose(xtemp);
smtObj.States(1:size(ytemp,2),2) = transpose(ytemp);
smtObj.States(1:size(ytemp,2),3) = zeros(size(ytemp,2),1);
%
tempPlotPlanning(map,start,goal,pthObj,solnInfo,true);
%smp=plot(xtemp,ytemp,'g:','LineWidth',1.7);

legend('planned path','smooth path');

%% get tracking result
clc;
% robotpathObj = mecanumTracking(start, goal, pthObj, 1, 0.5);
% robotpathObj_filtered = mecanumTracking(start, smtObj.States(end,:), smtObj, 0.1, 0.1); %0.24
% robotpathObj_filtered = mecanumTracking(start, pthObj.States(end,:), pthObj, 0.5, 0.2);
robotpathObj_raw = mecanumTracking(start, pthObj.States(end,:), pthObj, 0.9, 0.2); %0.24 %3
%% plot results
close all; cla;
% plotTrajectoryPlanning(map,smtObj,solnInfo, true, start, smtObj.States(end,:),robotpathObj);
% plotTrajectoryPlanning(map,pthObj,solnInfo, true, start, pthObj.States(end,:),robotpathObj_filtered1);
% plotTrajectoryPlanning(map,smtObj,solnInfo, true, start, smtObj.States(end,:),robotpathObj_filtered);
tempplotTrajectoryPlanning(map,pthObj, start, pthObj.States(end,:),robotpathObj_raw);
figure(1); hold on; plot(robotpathObj_raw.state(1,:),robotpathObj_raw.state(2,:),'g.');
disp(robotpathObj_raw.time(end));
%%
% plot(robotpathObj_filtered.state(1,:),robotpathObj_filtered.state(2,:),'g.');
plot(robotpathObj_raw.state(1,:),robotpathObj_raw.state(2,:),'g.');
%%
clc;
chagne_of_rate = RoC(robotpathObj_raw);
%figure(); hold on;
%%
cla;
% figure(); 
subplot(1,2,1); 
plot(robotpathObj_raw.time, robotpathObj_raw.statedot(1,:),'LineWidth',1.5); grid on; hold on;
plot(robotpathObj_filtered1.time, robotpathObj_filtered1.statedot(1,:),'LineWidth',1.5);
legend('SMC','SMC with filter', 'fontsize',14);
title('X-axis Velocity','fontsize',17, 'fontweight','bold'); xlabel('time[sec]','fontsize',15); ylabel('vel[m/s]','fontsize',15);
xlim([0 41]); ylim([-3 3]);

subplot(1,2,2);
plot(robotpathObj_raw.time, robotpathObj_raw.statedot(2,:),'LineWidth',1.5);grid on; hold on;
plot(robotpathObj_filtered1.time, robotpathObj_filtered1.statedot(2,:),'LineWidth',1.5);
legend('SMC','SMC with filter', 'fontsize',14);
title('Y-axis Velocity','fontsize',17, 'fontweight','bold'); xlabel('time[sec]','fontsize',15); ylabel('vel[m/s]','fontsize',15);
xlim([0 41]); ylim([-3 3]);

%%
close all; 
figure(); 
subplot(1,2,1); 
plot(robotpathObj_raw.time(1:end-1), robotpathObj_raw.errorDist,'--','LineWidth',1.5);hold on; grid on; 
plot(robotpathObj_filtered1.time(1:end-1), robotpathObj_filtered1.errorDist,'LineWidth',1.5);
legend('SMC', 'SMC with filter', 'fontsize',14);
title('Trajectory error','fontsize',17, 'fontweight','bold'); xlabel('time[sec]','fontsize',15); ylabel('e_d[m]','fontsize',15);
xlim([0 50]); ylim([0 3]);
hold off;

subplot(1,2,2); 
plot(robotpathObj_raw.time, robotpathObj_raw.wheelvel(1,:),'--','LineWidth',1.5);hold on; grid on; 
plot(robotpathObj_filtered1.time, robotpathObj_filtered1.wheelvel(1,:),'LineWidth',1.5);
legend('SMC', 'SMC with filter', 'fontsize',14);
title('First wheel velocity','fontsize',17, 'fontweight','bold'); xlabel('time[sec]','fontsize',15); ylabel('vel[deg/sec]','fontsize',15);
xlim([0 50]); ylim([-100 150]);
%%
robotpathObj_raw = mecanumTracking(start, smtObj.States(end,:), smtObj, 0.4, 0.26); %0.24
robotpathObj_raw1 = mecanumTracking(start, smtObj.States(end,:), smtObj, 0.4, 0.26); %0.24
close all;
%%
plotTrajectoryPlanning(map,smtObj,solnInfo, true, start, smtObj.States(end,:),robotpathObj_raw1);
plotTrajectoryPlanning(map,smtObj,solnInfo, true, start, smtObj.States(end,:),robotpathObj_raw);



function tempPlotPlanning(map,start,goal,pthObj,solnInfo,solnInfo_on)
% trajectory in map
show(map); hold on;
if solnInfo_on == true
    sol = plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-','color','[0.9 0.9 0.9]');
    sol.Annotation.LegendInformation.IconDisplayStyle = 'off';
end
plot(pthObj.States(:,1),pthObj.States(:,2),'b-.','LineWidth',0.7,'MarkerSize',4); % draw trajectory path
s = plot(start(1), start(2), 'o','MarkerSize',8,'MarkerEdgeColor','red','MarkerFaceColor','red');  % start point(circle)
s.Annotation.LegendInformation.IconDisplayStyle = 'off';
g = plot(goal(1), goal(2), 's','MarkerSize',8,'MarkerEdgeColor','k','MarkerFaceColor','green');    % goal point(square)
g.Annotation.LegendInformation.IconDisplayStyle = 'off';

title('Path Planning for Robot','fontsize',17,'fontweight','bold');
legend('planned path');
end

function tempplotTrajectoryPlanning(map,pthObj, start, goal, robotpathObj)
% trajectory in map
show(map); hold on;
plot(pthObj.States(:,1),pthObj.States(:,2),'b-.','LineWidth',0.7,'MarkerSize',4); % draw trajectory path
plot(start(1), start(2), 'o','MarkerSize',8,'MarkerEdgeColor','red','MarkerFaceColor','red');  % start point(circle)
plot(goal(1), goal(2), 's','MarkerSize',8,'MarkerEdgeColor','k','MarkerFaceColor','green');    % goal point(square)

% robot behavior 
figure();
subplot(2,2,1); % acceleration
plot(robotpathObj.time, robotpathObj.stateddot(1,:),'LineWidth',1.5); grid on; hold on;
plot(robotpathObj.time, robotpathObj.stateddot(2,:),'--','LineWidth',1.5);
legend('x axis', 'y axis','fontsize',14); 
title('Acceleration','fontsize',17, 'fontweight','bold'); xlabel('time[sec]','fontsize',15); ylabel('acc[m/s^2]','fontsize',15);
xlim([0 robotpathObj.time(end)]); ylim([-0.5 0.5]);
hold off;

subplot(2,2,2); % velocity
plot(robotpathObj.time, robotpathObj.statedot(1,:),'LineWidth',1.5); grid on; hold on;
plot(robotpathObj.time, robotpathObj.statedot(2,:),'--','LineWidth',1.5);
legend('x axis', 'y axis', 'fontsize',14);
title('Velocity','fontsize',17, 'fontweight','bold'); xlabel('time[sec]','fontsize',15); ylabel('vel[m/s]','fontsize',15);
xlim([0 robotpathObj.time(end)]); ylim([-1.3 1.3]);
hold off;

subplot(2,2,3); % final acceleration 
for i = 1:size(robotpathObj.time,2)
    acc_result(i) = sqrt(robotpathObj.stateddot(1,i)^2 + robotpathObj.stateddot(2,i)^2);
end
plot(robotpathObj.time, acc_result,'b'); grid on;
title('Acceleration Vector Norm','fontsize',17, 'fontweight','bold'); xlabel('time[sec]','fontsize',15); ylabel('acc[m/s^2]','fontsize',15);
xlim([0 robotpathObj.time(end)]); ylim([0 0.5]);
% legend('x axis', 'y axis','fontsize',14);


subplot(2,2,4); % final vel
for i = 1:size(robotpathObj.time,2)
    acc_result(i) = sqrt(robotpathObj.statedot(1,i)^2 + robotpathObj.statedot(2,i)^2);
end
% legend('x axis', 'y axis', 'fontsize',14);
plot(robotpathObj.time, acc_result,'b'); grid on;
title('Velocity Vector Norm','fontsize',17, 'fontweight','bold'); xlabel('time[sec]','fontsize',15); ylabel('vel[m/s]','fontsize',15);
xlim([0 robotpathObj.time(end)]); ylim([0 1.3]);
hold off;
% 
% figure();
% plot(robotpathObj.time(1:end-1), robotpathObj.svar(1,:)); hold on; grid on;
% plot(robotpathObj.time(1:end-1), robotpathObj.svar(2,:));
% plot(robotpathObj.time(1:end-1), robotpathObj.svar(3,:));
% title('Sliding surface','fontsize',17, 'fontweight','bold'); xlabel('time[sec]','fontsize',15); ylabel('s','fontsize',15);
% xlim([0 robotpathObj.time(end)]); %ylim([0 6]);
% hold off;
% 
% figure();
% plot(robotpathObj.time, 180/pi*robotpathObj.state(3,:)); grid on;
% title('Yaw angle','fontsize',17, 'fontweight','bold'); xlabel('time[sec]','fontsize',15); ylabel('yaw angle [deg/s]','fontsize',15);
% xlim([0 robotpathObj.time(end)]); %ylim([0 6]);
% hold off;

% figure();
% w_1 = plot(robotpathObj.time, robotpathObj.wheelvel(1,:),'r.','MarkerSize',5); hold on; grid on;
% w_2 = plot(robotpathObj.time, robotpathObj.wheelvel(2,:),'g.','MarkerSize',5);
% w_3 = plot(robotpathObj.time, robotpathObj.wheelvel(3,:),'b.','MarkerSize',5);
% w_4 = plot(robotpathObj.time, robotpathObj.wheelvel(4,:),'k.','MarkerSize',5);
end