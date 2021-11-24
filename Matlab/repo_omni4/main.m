clear all; clc; close all; cla;
% set map
map = drawMapOccupancy('factory02.pgm', 10);

% get planning result
start = [12 1 0];
goal = [12 24 0];

tstart = tic;
[pthObj,solnInfo] = RRTStarPlannerSum(map, start, goal, 0.2, 10000000, true);

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
%%
tempPlotPlanning(map,start,goal,pthObj,solnInfo,true);
smp=plot(xtemp,ytemp,'g:','LineWidth',1.7);

legend('planned path','smooth path');

%% get tracking result
% robotpathObj = mecanumTracking(start, goal, pthObj, 1, 0.5);
robotpathObj = mecanumTracking(start, smtObj.States(end,:), smtObj, 0.24, 0.1); %0.25

%% plot results
close all; cla;
plotTrajectoryPlanning(map,smtObj,solnInfo, true, start, smtObj.States(end,:),robotpathObj)

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