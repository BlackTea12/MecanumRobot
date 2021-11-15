% -------------------------------------------------------------------------
%
% File : DynamicWindowApproachSample.m
%
% Discription : Mobile Robot Motion Planning with Dynamic Window Approach
%
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2014 Atsushi Sakai
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------

function [] = DynamicWindowApproachSample()
 
close all;
clear all;
 
disp('Dynamic Window Approach sample program start!!')

x=[0 0 pi/2 0 0]';%[x(m),y(m),yaw(Rad),v(m/s),(rad/s)]
goal=[10,10];% [x(m),y(m)]
% [x(m) y(m)]
obstacle=[0 2;
          4 2;
          4 4;
          5 4;
          5 5;
          5 6;
          5 9
          8 8
          8 9
          7 9];
      
obstacleR=0.5;
global dt; dt=0.1;

Kinematic=[1.0,toRadian(20.0),0.2,toRadian(50.0),0.01,toRadian(1)];


evalParam=[0.1,0.2,0.1,3.0];
area=[-1 11 -1 11]; %[xmin xmax ymin ymax]
result.x=[];
tic;
%movcount=0;
% Main loop
for i=1:5000
    [u,traj]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR);
    x=f(x,u);
    result.x=[result.x; x'];
    
    if norm(x(1:2)-goal')<0.5
        disp('Arrive Goal!!');break;
    end
    
    %====Animation====
    hold off;
    ArrowLength=0.5;
    quiver(x(1),x(2),ArrowLength*cos(x(3)),ArrowLength*sin(x(3)),'ok');hold on;
    plot(result.x(:,1),result.x(:,2),'-b');hold on;
    plot(goal(1),goal(2),'*r');hold on;
    plot(obstacle(:,1),obstacle(:,2),'*k');hold on;
    if ~isempty(traj)
        for it=1:length(traj(:,1))/5
            ind=1+(it-1)*5;
            plot(traj(ind,:),traj(ind+1,:),'-g');hold on;
        end
    end
    axis(area);
    grid on;
    drawnow;
    %movcount=movcount+1;
    %mov(movcount) = getframe(gcf);
end
figure(2)
plot(result.x(:,4));
toc
%movie2avi(mov,'movie.avi');
 

function [u,trajDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R)
%Dynamic Window[vmin,vmax,min,max]
Vr=CalcDynamicWindow(x,model);
[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam);

if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0];return;
end

evalDB=NormalizeEval(evalDB);

feval=[];
for id=1:length(evalDB(:,1))
    feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
end
evalDB=[evalDB feval];

[maxv,ind]=max(feval);
u=evalDB(ind,1:2)';

function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam)
evalDB=[];
trajDB=[];

for vt=Vr(1):model(5):Vr(2)
    for ot=Vr(3):model(6):Vr(4)
        [xt,traj]=GenerateTrajectory(x,vt,ot,evalParam(4),model);
        heading=CalcHeadingEval(xt,goal);
        dist=CalcDistEval(xt,ob,R);
        vel=abs(vt);
        
        evalDB=[evalDB;[vt ot heading dist vel]];
        trajDB=[trajDB;traj];     
    end
end

function EvalDB=NormalizeEval(EvalDB)
if sum(EvalDB(:,3))~=0
    EvalDB(:,3)=EvalDB(:,3)/sum(EvalDB(:,3));
end
if sum(EvalDB(:,4))~=0
    EvalDB(:,4)=EvalDB(:,4)/sum(EvalDB(:,4));
end
if sum(EvalDB(:,5))~=0
    EvalDB(:,5)=EvalDB(:,5)/sum(EvalDB(:,5));
end

function [x,traj]=GenerateTrajectory(x,vt,ot,evaldt,model)
global dt;
time=0;
u=[vt;ot];
traj=x;
while time<=evaldt
    time=time+dt;
    x=f(x,u);
    traj=[traj x];
end

function stopDist=CalcBreakingDist(vel,model)
global dt;
stopDist=0;
while vel>0
    stopDist=stopDist+vel*dt;
    vel=vel-model(3)*dt;
end

function dist=CalcDistEval(x,ob,R)
dist=2;
for io=1:length(ob(:,1))
    disttmp=norm(ob(io,:)-x(1:2)')-R;
    if dist>disttmp
        dist=disttmp;
    end
end

function heading=CalcHeadingEval(x,goal)

theta=toDegree(x(3));
goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));

if goalTheta>theta
    targetTheta=goalTheta-theta;
else
    targetTheta=theta-goalTheta;
end

heading=180-targetTheta;

function Vr=CalcDynamicWindow(x,model)
global dt;
Vs=[0 model(1) -model(2) model(2)];

Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];

Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];

function x = f(x, u)
% Motion Model
global dt;
 
F = [1 0 0 0 0
     0 1 0 0 0
     0 0 1 0 0
     0 0 0 0 0
     0 0 0 0 0];
 
B = [dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt
    1 0
    0 1];

x= F*x+B*u;

function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;

function degree = toDegree(radian)
% radian to degree
degree = radian/pi*180;