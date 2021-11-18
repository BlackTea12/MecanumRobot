% close all; clear all; clc;
% R = 0.05;   % wheel radius [m]
% J1 = 0.095;%0.000625;  % nominal inertial of wheel [kgm^2]
% H = [0 J1/2 0 J1/2;
%      -J1/2 0 -J1/2 0;
%      0 J1/2 0 J1/2;
%      -J1/2 0 -J1/2 0];
% d = 1;%25; %[Nm] 
% B = [8*d 0 0; 0 8*d 0; 0 0 4*d/0.3];
% 
% p = 5;
% q = 3;
% lamda1 = 1.5;
% lamda2 = 1.5;
% lamda3 = 1.5;
% lamda = [lamda1 0 0; 0 lamda2 0; 0 0 lamda3];
% dt = 0.01;
% 
% cur_state.state = [0;0;0];
% cur_state.statedot = [0;0;0];
% cur_state.state2dot = [0;0;0];
% 
% des_state.state = [1;1;0];
% des_state.statedot = [0;0;0];
% des_state.prestatedot = [0;0;0];
% 
% err.state = [0;0;0];
% err.prestate = [0;0;0];
% 
% figure(1); hold on; grid on;
% % xlim([-0.5 10]); ylim([-160 160]);
% xlim([-5 5]); ylim([-5 5]);
% for i=0:dt:10
%     %des_state.state = [dt*i;dt*i;deg2rad(45)];
%     error = cur_state.state - des_state.state;
%     errdot = (err.state - err.prestate)/dt;
%     
%     Qr2dot = (des_state.statedot - des_state.prestatedot)/dt;
%     
%     temp_errdot_pq = [errdot(1)^(p/q);errdot(2)^(p/q);errdot(3)^(p/q)];
%     s = error + lamda*temp_errdot_pq;
%     
%     % important!!
% %     M_inv = [0.001 0 0;
% %             0 0.001 0;
% %             0 0 0.001];
%     M_inv = [0 0 0;
%             0 0 0;
%             0 0 0];    
%     if(errdot(1) ~= 0)
%         M_inv(1,1) = q/(p*lamda1)*errdot(1)^(1-(p/q));
%     end
%     if(errdot(2) ~= 0)
%         M_inv(2,2) = q/(p*lamda2)*errdot(2)^(1-(p/q));
%     end
%     if(errdot(3) ~= 0)
%         M_inv(3,3) = q/(p*lamda3)*errdot(3)^(1-(p/q));
%     end
%     
%     hpsi_inv_mat = hpsi_inv(cur_state.state(3));   
%     
%     u_eq = 4*J1/R * hpsi_inv_mat * (Qr2dot- M_inv*errdot);
%     u_r = -hpsi_inv_mat * B * sign(s);
% 
%     % final control input
%     ctr_in = u_eq + u_r;
%     
%     % ctr input to plant model
%     hpsi_mat = h_psi(cur_state.state(3));
%     cur_state.state2dot = R/(4*J1) * hpsi_mat * ctr_in;
%     
%     % data saving
%     cur_state.statedot = cur_state.statedot + dt*cur_state.state2dot;
%     cur_state.state = cur_state.state + dt*cur_state.statedot;
%     
%     % draw now
%     % trajectory
%     p_tr = plot(cur_state.state(1),cur_state.state(2),'ro');
%     txt = text(9,9,string(i));
%     drawnow;
%     delete(txt); 
%     delete(p_tr);   
%     
%     % ctr input
% %     plot(i, u_r(1),'ro');%,i, ctr_in(2),'o',i, ctr_in(3),'o',i, ctr_in(4),'o');
% %     txt = text(1,1,string(i));
% %     drawnow;
% %     %delete(p_ctr); 
% %     delete(txt);
% end
% 
% function mat = hpsi_inv(psi)
% psi = psi + pi/4;
% sqrt2 = sqrt(2);
% mat = 0.25 * [-sqrt2*sin(psi) sqrt2*cos(psi) 0.3;
%                 sqrt2*cos(psi) sqrt2*sin(psi) -0.3;
%                 -sqrt2*sin(psi) sqrt2*cos(psi) -0.3;
%                 sqrt2*cos(psi) sqrt2*sin(psi) 0.3];
% end
% 
% function mat = h_psi(psi)
% % returns psi 3x4 matrix
% psi = psi + pi/4;
% root2 = sqrt(2);
% 
% % third row should be 1/(a+b) which will be 1/(2*rou)
% mat = [-root2*sin(psi) root2*cos(psi) -root2*sin(psi) root2*cos(psi);
%         root2*cos(psi) root2*sin(psi) root2*cos(psi) root2*sin(psi);
%         10/3 -10/3 -10/3 10/3];
% end
% 
% function wheelVel= getwheelvel(psi, cur_state_dot)
% R = 0.05;   % wheel radius [m]
% hpsi_inv_mat = hpsi_inv(psi);
% 
% wheelVel = 4/R* hpsi_inv_mat * cur_state_dot;
% end

%% discrete conventional smc
%close all; 
clear all; clc; cla;
R = 0.05;   % wheel radius [m]
J1 = 0.095;%0.000625;  % nominal inertial of wheel [kgm^2]
H = [0 J1/2 0 J1/2;
     -J1/2 0 -J1/2 0;
     0 J1/2 0 J1/2;
     -J1/2 0 -J1/2 0];
d = 25; %[Nm] 
lamda1 = 8;
lamda2 = 8;
lamda3 = 8;
lamda = [lamda1 0 0; 0 lamda2 0; 0 0 lamda3];
N = diag([8*d;8*d;4*d/0.3]);
dt = 0.01;

% initialize
cur_state.state = [0;0;0];
cur_state.statedot = [0;0;0];
cur_state.state2dot = [0;0;0];

des_state.state = [8;5;0];
des_state.statedot = [0;0;0];
des_state.prestatedot = [0;0;0];

err.state = [0;0;0];
err.prestate = [0;0;0];

slope = 1.5;
radius = 4.5;

% data storage
result.time = 0;
result.state = [0;0;0];
result.statedot = [0;0;0];
result.stateddot = [0;0;0];
result.wheeldeg = [0;0;0;0];
result.wheelvel = [0;0;0;0];
result.svar = [0;0;0];

figure(1); hold on; grid on;
title('trajectory','fontsize',14,'fontweight','bold');
xlabel('X[m]','fontsize',12); ylabel('Y[m]','fontsize',12);
t = 0:0.1:10;
% plot(radius*sin(t), radius*cos(t),'g','LineWidth',2); % circle trajectory
plot(t, 2+t*slope,'g','LineWidth',2); % slope trajectory
legend('reference path', 'Location', 'Best','fontsize',13);

% xlim([-0.5 10]); ylim([-160 160]);  % control input u limit
% xlim([-10 10]); ylim([-10 10]); % trajectory limit
xlim([0 8]); ylim([0 16]); % slope trajectory limit
%xlim([-6 6]); ylim([-6 6]); % circle trajectory limit
cnt = 2;
%%
filename = 'slopeTrajectory.gif';
for i=dt:dt:6
    % ramp input,
%     des_state.state = [i;2+slope*i;0];
%     des_state.statedot = [1;slope;0];

    % circle input
    des_state.state = [radius*cos(i); radius*sin(i);0];
    des_state.statedot = [-radius*sin(i); radius*cos(i);0];
    
    % error calculation
    err.state = - cur_state.state + des_state.state;
    errdot = (err.state - err.prestate)/dt;
    
    % state ddot calculation
    Qr2dot = (des_state.statedot - des_state.prestatedot)/dt;
    
    % conventional sliding mode variable calculation
    s = errdot + lamda*err.state;    
    hpsi_inv_mat = hpsi_inv(cur_state.state(3));   

    % final control input
    ctr_in = hpsi_inv_mat*(4*J1/R*(Qr2dot+lamda*errdot)+N*sign(s));
    
    % ctr input to plant model
    hpsi_mat = h_psi(cur_state.state(3));
    cur_state.state2dot = R/(4*J1) * hpsi_mat * ctr_in;
    
    % wheel velocity calculation
    wheelVel= getwheelvel(cur_state.state(3), cur_state.statedot);
    
    % store data
    result.time(cnt) = i;
    result.state(:,cnt) = cur_state.state;
    result.statedot(:,cnt) = cur_state.statedot;
    result.stateddot(:,cnt) = cur_state.state2dot;
    result.wheeldeg(:,cnt) = result.wheeldeg(:,cnt-1)+wheelVel*dt;
    result.wheelvel(:,cnt) = wheelVel;
    result.svar(:,cnt-1) = s;
    result.errorDist(cnt-1) = sqrt(err.state(1)^2+err.state(2)^2);
    cnt = cnt+1;
    
    % data saving
    cur_state.statedot = cur_state.statedot + dt*cur_state.state2dot;
    cur_state.state = cur_state.state + dt*cur_state.statedot;

    % pre data saving
    des_state.prestatedot = des_state.statedot;
    err.prestate = err.state;
    
    % draw now
    % trajectory
%     p_tr = plot(cur_state.state(1),cur_state.state(2),'r.');
% %     txt = text(4,5,'t: '+string(i)+'(s)');  % circle
%     txt = text(1,14,'t: '+string(i)+'(s)');  % slope
%     p_tr.Annotation.LegendInformation.IconDisplayStyle = 'off';
%     drawnow;
%     delete(txt); 
    %delete(p_tr);   
    
    % error
%     plot(i, err.state(1),'r.');%,i, ctr_in(2),'o',i, ctr_in(3),'o',i, ctr_in(4),'o');
%     txt = text(1,1,string(i));
%     drawnow;
%     delete(txt);

    % ctr input
%     plot(i, ctr_in(1),'r.');%,i, ctr_in(2),'o',i, ctr_in(3),'o',i, ctr_in(4),'o');
%     txt = text(1,1,string(i));
%     drawnow;
%     delete(txt);
    
    % wheel velocity
%     plot(i, rad2deg(wheelVel(2)),'r.');%,i, ctr_in(2),'o',i, ctr_in(3),'o',i, ctr_in(4),'o');
%     txt = text(1,1,string(i));
%     drawnow;
%     delete(txt);

    % save as .gif
%     frame = getframe(1);
%     img = frame2im(frame);
%     [imind cm] = rgb2ind(img,256);
%     if i == dt
%         imwrite(imind,cm,filename,'gif','Loopcount',1,'DelayTime',1/18);
%     else
%         imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',1/18);
%     end
%     delete(txt); 
end

%% using function to track
robotpathObj = mecanumTracking(start, goal, pthObj, 3, 0.05);

%% result plot
% figure(2);
% plot(result.time, result.stateddot(1,:),'LineWidth',1.5); grid on; hold on;
% plot(result.time, result.stateddot(2,:),'--','LineWidth',1.5); 
% %ylim([-1.5 1.5]);
% xlabel('time[sec]'); ylabel('acc[m/s^2]');
% title('Acceleration of mobile robot','fontsize',14, 'fontweight','bold');
% hold off;
figure(1);


subplot(2,2,1); % acc
plot(robotpathObj.time, robotpathObj.stateddot(1,:),'LineWidth',1.5); grid on; hold on;
plot(robotpathObj.time, robotpathObj.stateddot(2,:),'--','LineWidth',1.5);
legend('x axis', 'y axis','fontsize',14); 
title('Acceleration','fontsize',17, 'fontweight','bold'); xlabel('time[sec]','fontsize',15); ylabel('acc[m/s^2]','fontsize',15);
xlim([0 83]); ylim([-4 4]);
hold off;

subplot(2,2,2); % vel
plot(robotpathObj.time, robotpathObj.statedot(1,:),'LineWidth',1.5); grid on; hold on;
plot(robotpathObj.time, robotpathObj.statedot(2,:),'--','LineWidth',1.5);
legend('x axis', 'y axis', 'fontsize',14);
title('Velocity','fontsize',17, 'fontweight','bold'); xlabel('time[sec]','fontsize',15); ylabel('vel[m/s]','fontsize',15);
xlim([0 83]); ylim([-5 5]);
hold off;

subplot(2,2,3); % final acc 
for i = 1:size(robotpathObj.time,2)
    acc_result(i) = sqrt(robotpathObj.stateddot(1,i)^2 + robotpathObj.stateddot(2,i)^2);
end
plot(robotpathObj.time, acc_result,'b'); grid on;
title('Acceleration Vector Norm','fontsize',17, 'fontweight','bold'); xlabel('time[sec]','fontsize',15); ylabel('acc[m/s^2]','fontsize',15);
xlim([0 83]); ylim([-1 4]);
% legend('x axis', 'y axis','fontsize',14);


subplot(2,2,4); % final vel
for i = 1:size(robotpathObj.time,2)
    acc_result(i) = sqrt(robotpathObj.statedot(1,i)^2 + robotpathObj.statedot(2,i)^2);
end
% legend('x axis', 'y axis', 'fontsize',14);
plot(robotpathObj.time, acc_result,'b'); grid on;
title('Velocity Vector Norm','fontsize',17, 'fontweight','bold'); xlabel('time[sec]','fontsize',15); ylabel('vel[m/s]','fontsize',15);
xlim([0 83]); ylim([0 6]);
hold off;

% subplot(2,2,[3 4]); % error
% plot(robotpathObj.time(1:end-1), robotpathObj.errorDist, 'LineWidth', 1.5); grid on;
% title('Tracking Error'); xlabel('time[sec]','fontsize',15); ylabel('error[m]','fontsize',15);

%----------------------------------------------------------------------------------%
% given start and goal point and pathObj
% with refDisp which is reference distance point
% dependent on mobile robot's position
% it will follow reference path objects
function robotpathObj = mecanumTracking(start, goal, pthObj, refDisp, valdisGoal)
disp("Start Tracking...");
%-------------------------robot's parameter---------------------------------------%
R = 0.05;   % wheel radius [m]
J1 = 0.095;%0.000625;  % nominal inertial of wheel [kgm^2]
H = [0 J1/2 0 J1/2;
     -J1/2 0 -J1/2 0;
     0 J1/2 0 J1/2;
     -J1/2 0 -J1/2 0];
d = 25; %[Nm] 
lamda1 = 8;
lamda2 = 8;
lamda3 = 8;
lamda = [lamda1 0 0; 0 lamda2 0; 0 0 lamda3];
N = diag([8*d;8*d;4*d/0.3]);
%---------------------------------------------------------------------------------%

dt = 0.01;  % time variable

% initialize
cur_state.state = transpose(start);
cur_state.statedot = [0;0;0];
cur_state.state2dot = [0;0;0];

des_state.state = [0;0;0];
des_state.statedot = [0;0;0];
des_state.prestatedot = [0;0;0];

err.state = [0;0;0];
err.prestate = [0;0;0];

% data storage
robotpathObj.time = 0;
robotpathObj.state = [0;0;0];
robotpathObj.statedot = [0;0;0];
robotpathObj.stateddot = [0;0;0];
robotpathObj.wheeldeg = [0;0;0;0];
robotpathObj.wheelvel = [0;0;0;0];
robotpathObj.svar = [0;0;0];

index = 1;
length = size(pthObj.States,1);
cnt = 2;

% start tracking
while true
    % deciding desired point based on refDisp
    if index == length
        des_state.state = transpose(pthObj.States(index,:));
    else
        for j = index:length
            temp = cur_state.state - transpose(pthObj.States(j,:)); temp(3,1) = 0;
            pos =  norm(temp);
            if pos >= refDisp
                index = j;
                des_state.state = transpose(pthObj.States(j,:));
                break;
            end
        end
    end
    
    % error calculation
    err.state = - cur_state.state + des_state.state;
    errdot = (err.state - err.prestate)/dt;
    
    % state ddot calculation
    Qr2dot = (des_state.statedot - des_state.prestatedot)/dt;
    
    % conventional sliding mode variable calculation
    s = errdot + lamda*err.state;    
    hpsi_inv_mat = hpsi_inv(cur_state.state(3));   

    % final control input
    ctr_in = hpsi_inv_mat*(4*J1/R*(Qr2dot+lamda*errdot)+N*sign(s));
    
    % ctr input to plant model
    hpsi_mat = h_psi(cur_state.state(3));
    cur_state.state2dot = R/(4*J1) * hpsi_mat * ctr_in;
    
    % x-y acceleration limit(unfinished)
    acc_limit = 2.5;    % [10m/s^2]
    indicator_acc = norm(cur_state.state2dot(1:2,1));
    if indicator_acc >= acc_limit
        gain = acc_limit / indicator_acc;
        ctr_in = ctr_in*gain;
        cur_state.state2dot = R/(4*J1) * hpsi_mat * ctr_in;
    end
    
    % wheel velocity calculation
    wheelVel= getwheelvel(cur_state.state(3), cur_state.statedot);
    
    % store data
    robotpathObj.time(cnt) = robotpathObj.time(cnt-1) + dt;
    robotpathObj.state(:,cnt) = cur_state.state;
    robotpathObj.statedot(:,cnt) = cur_state.statedot;
    robotpathObj.stateddot(:,cnt) = cur_state.state2dot;
    robotpathObj.wheeldeg(:,cnt) = robotpathObj.wheeldeg(:,cnt-1)+wheelVel*dt;
    robotpathObj.wheelvel(:,cnt) = wheelVel;
    robotpathObj.svar(:,cnt-1) = s;
    robotpathObj.errorDist(cnt-1) = sqrt(err.state(1)^2+err.state(2)^2);
    
    % data saving
    cur_state.statedot = cur_state.statedot + dt*cur_state.state2dot;
    cur_state.state = cur_state.state + dt*cur_state.statedot;

    % pre data saving
    des_state.prestatedot = des_state.statedot;
    err.prestate = err.state;
    
    % check for goal arrival
    arrivalPos = transpose(goal)-cur_state.state; arrivalPos(3,1) = 0;
    if norm(arrivalPos) < valdisGoal
        break;
    end
    
    cnt = cnt+1;
end
disp("Tracking Finished...");
end

% matrix dependent on mobile robot's yaw angle
function mat = hpsi_inv(psi)
psi = psi + pi/4;
sqrt2 = sqrt(2);
mat = 0.25 * [-sqrt2*sin(psi) sqrt2*cos(psi) 0.3;
                sqrt2*cos(psi) sqrt2*sin(psi) -0.3;
                -sqrt2*sin(psi) sqrt2*cos(psi) -0.3;
                sqrt2*cos(psi) sqrt2*sin(psi) 0.3];
end

% inverse matrix dependent on mobile robot's yaw angle
function mat = h_psi(psi)
% returns psi 3x4 matrix
psi = psi + pi/4;
sqrt2 = sqrt(2);

% third row should be 1/(a+b) which will be 1/(2*rou)
mat = [-sqrt2*sin(psi) sqrt2*cos(psi) -sqrt2*sin(psi) sqrt2*cos(psi);
        sqrt2*cos(psi) sqrt2*sin(psi) sqrt2*cos(psi) sqrt2*sin(psi);
        10/3 -10/3 -10/3 10/3];
end

% wheel velocity calculator (rad/sec), dependent on current mobile robot's
% state
function wheelVel= getwheelvel(psi, cur_state_dot)
R = 0.05;   % wheel radius [m]
hpsi_inv_mat = hpsi_inv(psi);

wheelVel = 4/R* hpsi_inv_mat * cur_state_dot;
end