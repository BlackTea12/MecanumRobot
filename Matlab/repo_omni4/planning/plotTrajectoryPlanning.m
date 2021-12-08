% plot Occupancy map based results
function plotTrajectoryPlanning(map,pthObj,solnInfo,solnInfo_on, start, goal, robotpathObj)
% trajectory in map
show(map); hold on;
if solnInfo_on == true
    plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-','color','[0.9 0.9 0.9]');
end
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

figure();
plot(robotpathObj.time(1:end-1), robotpathObj.svar(1,:)); hold on; grid on;
plot(robotpathObj.time(1:end-1), robotpathObj.svar(2,:));
plot(robotpathObj.time(1:end-1), robotpathObj.svar(3,:));
title('Sliding surface','fontsize',17, 'fontweight','bold'); xlabel('time[sec]','fontsize',15); ylabel('s','fontsize',15);
xlim([0 robotpathObj.time(end)]); %ylim([0 6]);
hold off;

figure();
plot(robotpathObj.time, 180/pi*robotpathObj.state(3,:)); grid on;
title('Yaw angle','fontsize',17, 'fontweight','bold'); xlabel('time[sec]','fontsize',15); ylabel('yaw angle [deg/s]','fontsize',15);
xlim([0 robotpathObj.time(end)]); %ylim([0 6]);
hold off;

% figure();
% w_1 = plot(robotpathObj.time, robotpathObj.wheelvel(1,:),'r.','MarkerSize',5); hold on; grid on;
% w_2 = plot(robotpathObj.time, robotpathObj.wheelvel(2,:),'g.','MarkerSize',5);
% w_3 = plot(robotpathObj.time, robotpathObj.wheelvel(3,:),'b.','MarkerSize',5);
% w_4 = plot(robotpathObj.time, robotpathObj.wheelvel(4,:),'k.','MarkerSize',5);
end