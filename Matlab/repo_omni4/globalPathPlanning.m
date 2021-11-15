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
image2 = imread('workspace_ex02.pgm');
imageNorm = double(image2)/255;
imageOccupancy = 1 - imageNorm;
map2 = occupancyMap(imageOccupancy,20);
show(map2)
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
close all;
map2 = occupancyMap(imageOccupancy,10);
sv.Map = map2;
sv.ValidationDistance = 0.01;
ss.StateBounds = [map2.XWorldLimits; map2.YWorldLimits; [pi pi]];
planner = plannerRRTStar(ss,sv);
planner.ContinueAfterGoalReached = false;
planner.MaxIterations = 10000;
planner.MaxConnectionDistance = 0.6;
show(map2)
start = [2, 18, 0]; goal = [14, 10, 0];
rng(100,'twister');
[pthObj,solnInfo] = plan(planner,start,goal);
hold on;
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2); % draw path