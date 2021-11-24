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

%% map callback
close all; clear all; clc;
image1 = imread('factory01.pgm');
imshow(image1);
% make clean floor
len = size(image1);
for i = 1:len(1)
    for j = 1:len(2)
        if image1(i,j) < 10
            image1(i,j) = 0;
        end
    end
end
imageNorm = double(image1)/255;
imageOccupancy = imageNorm; % revert black and white
map2 = occupancyMap(imageOccupancy,1);

% figure(1);
% show(map2);

% image2 = imread('workspace_ex02.pgm');
% imageNorm = double(image2)/255;
% imageOccupancy = 1 - imageNorm;
% map2 = occupancyMap(imageOccupancy,10);

% ss = stateSpaceSE2;
% sv = validatorOccupancyMap(ss);
% close all;
% map2 = occupancyMap(imageOccupancy,10);
% sv.Map = map2;
% sv.ValidationDistance = 0.01;
% ss.StateBounds = [map2.XWorldLimits; map2.YWorldLimits; [pi pi]];

bounds = [map2.XWorldLimits; map2.YWorldLimits; [pi/2 pi/2]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.1;
sv = validatorOccupancyMap(ss);
sv.Map = map2;
sv.ValidationDistance = 0.01;

% rrt star planner
planner = plannerRRTStar(ss,sv);
planner.ContinueAfterGoalReached = true; %optimization
planner.MaxIterations = 800000;
planner.MaxConnectionDistance = 1;    %[m]

% a star planner
planner_compare = plannerAStarGrid(map2);
st = [650,210];
goa = [350,275];
plan(planner_compare,st,goa);
close all; show(planner_compare);

% figure(2);
show(map2)
start = [210, 115, 0]; goal = [278, 340, 0]; %[396, 400, 0];
rng(100,'twister');
%startT = tic();    % start timer
[pthObj,solnInfo] = plan(planner,start,goal);
%endT = toc(startT);    % end timer

%%
hold on; grid on;
plot(start(1), start(2), 'o','MarkerSize',10,'MarkerEdgeColor','red','MarkerFaceColor','magenta');
plot(goal(1), goal(2), 's','MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b');
%plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',1); % draw path
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

%% plot final results, tracking and planning
traj_len = size(robotpathObj.state, 2);
% close all;clc;
% % filename = 'pathtracking01.gif';
% show(map2)
% plot(start(1), start(2), 'o','MarkerSize',10,'MarkerEdgeColor','red','MarkerFaceColor','magenta');
% plot(goal(1), goal(2), 's','MarkerSize',10,'MarkerEdgeColor','k','MarkerFaceColor','b');
% plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',0.7); % draw path
% filename = 'track1123.gif';
cnt = 1;
for i=1:traj_len-1
    % save as .gif
%     frame = getframe(1);
%     img = frame2im(frame);
%     [imind cm] = rgb2ind(img,256);
%     if i == 1
%         imwrite(imind,cm,filename,'gif','Loopcount',1,'DelayTime',1/18);
%     else
%         imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',1/18);
%     end
    p_tr = plot(robotpathObj.state(1,i+1),robotpathObj.state(2,i+1),'g.');%o','MarkerSize',7, 'MarkerFace','green');
    p_reftr = plot(robotpathObj.refstate(1,i+1),robotpathObj.refstate(2,i+1),'bo');
    tempvel = [robotpathObj.statedot(1,i+1),robotpathObj.statedot(2,i+1)];
    vel = text(100,80,'v: '+string(norm(tempvel))+'(m/s)');
    txt = text(100,100,'t: '+string(robotpathObj.time(i))+'(s)');
    p_tr.Annotation.LegendInformation.IconDisplayStyle = 'off';
    p_reftr.Annotation.LegendInformation.IconDisplayStyle = 'off';
    drawnow;
    % saveGIF(cnt, filename, 18,2);
    delete(txt); 
    delete(vel);
    delete(p_reftr);
    %delete(p_tr);  
    cnt = cnt + 1;
end
%plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',0.7); % draw path

% summary of rrt star planner
function [pthObj,solnInfo] = RRTStarPlannerSum(map, start, goal, connectionDis, iterations)
bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.3;
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 0.1;

planner = plannerRRTStar(ss,sv);
planner.ContinueAfterGoalReached = true; %optimization
planner.MaxIterations = iterations;
planner.MaxConnectionDistance = connectionDis;    %[m]

[pthObj,solnInfo] = plan(planner,start,goal);
end

% str_pgm will have a format of string
% ex: test.pgm
function map = drawMapOccupancy(str_pgm, resolution)
image = imread(str_pgm);
% imshow(image);

% make clean floor
len = size(image);

for i = 1:len(1)
    for j = 1:len(2)
        if image(i,j) < 10
            image(i,j) = 0;
        end
    end
end
imageNorm = double(image)/255;
imageOccupancy = imageNorm; % revert black and white
map = occupancyMap(imageOccupancy,resolution);
end