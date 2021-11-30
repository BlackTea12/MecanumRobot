% given start and goal point and pathObj
% with refDisp which is reference distance point
% dependent on mobile robot's position
% it will follow reference path objects
function robotpathObj = mecanumTracking(start, goal, pthObj, refDisp, valdisGoal)
disp("Start Tracking!...");
%-------------------------robot's parameter---------------------------------------%
R = 0.05;   % wheel radius [m]
J1 = 0.095;%0.000625;  % nominal inertial of wheel [kgm^2]
H = [0 J1/2 0 J1/2;
     -J1/2 0 -J1/2 0;
     0 J1/2 0 J1/2;
     -J1/2 0 -J1/2 0];
d = 25; %[Nm] 
lamda1 = 0.5;%5;   %8 %0.028
lamda2 = 0.5;%5;   %8 %0.028
lamda3 = 0.5;%5;   %8 %0.028
lamda = [lamda1 0 0; 0 lamda2 0; 0 0 lamda3];
N = diag([8*d;8*d;4*d/0.3]);
%---------------------------------------------------------------------------------%

dt = 1/15;  % time variable, 15hz

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
robotpathObj.refstate = [0;0;0];
robotpathObj.state = [0;0;0];
robotpathObj.statedot = [0;0;0];
robotpathObj.stateddot = [0;0;0];
robotpathObj.wheeldeg = [0;0;0;0];
robotpathObj.wheelvel = [0;0;0;0];
robotpathObj.svar = [0;0;0];

index = 1;
length = size(pthObj.States,1);
cnt = 2;
tau = 1/6; %0.01 
ctr_filter_length = 2;
ctr_filter_pre = zeros(3,ctr_filter_length); % bigger the index, closer to the past data

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
    
%     % low pass filter
%     temp_ctr = (tau*robotpathObj.stateddot(:,cnt-1) + dt*cur_state.state2dot)/(tau+dt);
%     
%     % moving average filter
%     for k=1:ctr_filter_length
%         cur_state.state2dot = cur_state.state2dot + ctr_filter_pre(:,k);
%     end
%     cur_state.state2dot = temp_ctr / (size(ctr_filter_pre,2)+1);
%     
%     for k=0:ctr_filter_length-2
%         ctr_filter_pre(:,ctr_filter_length-k) = ctr_filter_pre(:,ctr_filter_length-1-k);
%     end
%     ctr_filter_pre(:,1) = cur_state.state2dot;
%     
%     
    % x-y acceleration limit(unfinished)
    acc_limit = 4;%5.3;%0.85;    % 0.25
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
    robotpathObj.refstate(:,cnt) = des_state.state;
    robotpathObj.state(:,cnt) = cur_state.state;
    robotpathObj.statedot(:,cnt) = cur_state.statedot;
    robotpathObj.stateddot(:,cnt) = cur_state.state2dot;
    robotpathObj.wheeldeg(:,cnt) = robotpathObj.wheeldeg(:,cnt-1)+wheelVel*dt;
    robotpathObj.wheelvel(:,cnt) = wheelVel;
    robotpathObj.svar(:,cnt-1) = s;
%     robotpathObj.errorDist(cnt-1) = sqrt(err.state(1)^2+err.state(2)^2);
    
    % calculation of error distance
    mindis = 10;
    for j=1:size(pthObj.States,1)
        temp = transpose(cur_state.state(1:2))-pthObj.States(j,1:2);
        if mindis > norm(temp)
            mindis = norm(temp);
        end
    end
    robotpathObj.errorDist(cnt-1) = mindis;
    
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
disp("Tracking Finished!");
end