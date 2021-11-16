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
close all; clear all; clc;
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

cur_state.state = [0;0;0];
cur_state.statedot = [0;0;0];
cur_state.state2dot = [0;0;0];

des_state.state = [0;5;0];
des_state.statedot = [0;0;0];
des_state.prestatedot = [0;0;0];

err.state = [0;0;0];
err.prestate = [0;0;0];

figure(1); hold on; grid on;
% xlim([-0.5 10]); ylim([-160 160]);
xlim([-10 10]); ylim([-10 10]);
for i=0:dt:10
    %des_state.state = [dt*i;dt*i;deg2rad(45)];
    error = - cur_state.state + des_state.state;
    errdot = (err.state - err.prestate)/dt;
    
    Qr2dot = (des_state.statedot - des_state.prestatedot)/dt;
    
    s = errdot + lamda*error;

    
    hpsi_inv_mat = hpsi_inv(cur_state.state(3));   

    % final control input
    ctr_in = hpsi_inv_mat*(4*J1/R*(Qr2dot+lamda*errdot)+N*sign(s));
    
    % ctr input to plant model
    hpsi_mat = h_psi(cur_state.state(3));
    cur_state.state2dot = R/(4*J1) * hpsi_mat * ctr_in;
    
    % data saving
    cur_state.statedot = cur_state.statedot + dt*cur_state.state2dot;
    cur_state.state = cur_state.state + dt*cur_state.statedot;
    
    % draw now
    % trajectory
    p_tr = plot(cur_state.state(1),cur_state.state(2),'ro');
    txt = text(9,9,string(i));
    drawnow;
    delete(txt); 
    delete(p_tr);   
    
    % ctr input
%     plot(i, u_r(1),'ro');%,i, ctr_in(2),'o',i, ctr_in(3),'o',i, ctr_in(4),'o');
%     txt = text(1,1,string(i));
%     drawnow;
%     %delete(p_ctr); 
%     delete(txt);
end

function mat = hpsi_inv(psi)
psi = psi + pi/4;
sqrt2 = sqrt(2);
mat = 0.25 * [-sqrt2*sin(psi) sqrt2*cos(psi) 0.3;
                sqrt2*cos(psi) sqrt2*sin(psi) -0.3;
                -sqrt2*sin(psi) sqrt2*cos(psi) -0.3;
                sqrt2*cos(psi) sqrt2*sin(psi) 0.3];
end

function mat = h_psi(psi)
% returns psi 3x4 matrix
psi = psi + pi/4;
root2 = sqrt(2);

% third row should be 1/(a+b) which will be 1/(2*rou)
mat = [-root2*sin(psi) root2*cos(psi) -root2*sin(psi) root2*cos(psi);
        root2*cos(psi) root2*sin(psi) root2*cos(psi) root2*sin(psi);
        10/3 -10/3 -10/3 10/3];
end

function wheelVel= getwheelvel(psi, cur_state_dot)
R = 0.05;   % wheel radius [m]
hpsi_inv_mat = hpsi_inv(psi);

wheelVel = 4/R* hpsi_inv_mat * cur_state_dot;
end