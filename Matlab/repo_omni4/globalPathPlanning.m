% resolution 0.1m
% map size 10m X 10m
% 
% map = binaryOccupancyMap(10,10,0.1);
% 
% function []=staticMapMaker(walls)
% % n number of walls
% % each wall has (center point, length, width) in 4x1 matrix
% % walls matrix is 4xn
% % plot the whole static map
% 
% n = length(walls); n = n(2);
% 
% figure(1);
% for i=1:n
%     
% end
% 
% end

%%
close all; clear all; clc;
image2 = imread('workspace_ex02.pgm');
imageNorm = double(image2)/255;
imageOccupancy = 1 - imageNorm;
map2 = occupancyMap(imageOccupancy,10);

% ss = stateSpaceSE2;
% sv = validatorOccupancyMap(ss);
% close all;
% map2 = occupancyMap(imageOccupancy,10);
% sv.Map = map2;
% sv.ValidationDistance = 0.01;
% ss.StateBounds = [map2.XWorldLimits; map2.YWorldLimits; [pi pi]];

bounds = [map2.XWorldLimits; map2.YWorldLimits; [-pi/2 pi/2]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.3;
sv = validatorOccupancyMap(ss);
sv.Map = map2;
sv.ValidationDistance = 0.05;


planner = plannerRRTStar(ss,sv);
planner.ContinueAfterGoalReached = true;
planner.MaxIterations = 10000;
planner.MaxConnectionDistance = 0.5;    %[m]
show(map2)
start = [2, 18, 0]; goal = [14, 10, 0];
rng(100,'twister');
%startT = tic();    % start timer
[pthObj,solnInfo] = plan(planner,start,goal);
%endT = toc(startT);    % end timer

hold on; grid on;
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2); % draw path
% endT
% flag = true;
% while(flag)
%     [pthObj,solnInfo] = plan(planner,start,goal);
%     f = plot(pthObj.States(:,1),pthObj.States(:,2),'r.-','LineWidth',2); % draw path
%     g = plot(pthObj.States(1,1),pthObj.States(1,2),'bo'); % draw now position of robot
%     start = [pthObj.States(2,1),pthObj.States(2,2), 0];
%     drawnow
%     % check arrival state
%     if(start(1) == goal(1))
%         if(start(2) == goal(2))
%             flag = false;
%         end
%     end
%     delete(f)
% end
