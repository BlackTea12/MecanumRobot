clear all; clc; close all; cla;
% set map
map = drawMapOccupancy('factory02.pgm', 10);

% get planning result
start = [12 1 0];
goal = [12 24 0];
[pthObj,solnInfo] = RRTStarPlannerSum(map, start, goal, 0.1, 500000);
%%
% get tracking result
robotpathObj = mecanumTracking(start, goal, pthObj, 1, 0.5);

% write for smoothing of path
%%
% plot results
plotTrajectoryPlanning(map,pthObj,solnInfo, true, start, goal,robotpathObj)