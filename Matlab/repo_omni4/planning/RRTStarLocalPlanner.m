% local planner
function [pthObj,solnInfo] = RRTStarLocalPlanner(local_map, start, goal, connectionDis, iterations, optimization)
bounds = [local_map.XWorldLimits; local_map.YWorldLimits; [-pi pi]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.3;
sv = validatorOccupancyMap(ss);
sv.Map = local_map;
sv.ValidationDistance = 0.05;

planner = plannerRRTStar(ss,sv);
planner.ContinueAfterGoalReached = optimization; %optimization
planner.MaxIterations = iterations;
planner.MaxConnectionDistance = connectionDis;    %[m]

[pthObj,solnInfo] = plan(planner,start,goal);

if isempty(pthObj)
    disp('No solutions for planning...');
else 
    disp('Solution found for planning!');
end

end