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

if isempty(pthObj)
    disp('No solutions for planning...');
else 
    disp('Solution found for planning!');
end

end