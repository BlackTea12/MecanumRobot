% 4 mecanum wheeled omni robot dynamic adding


% practical calculation of equation
m0 = 3.1;   % body mass [kg]
m1= 0.35;  % wheel mass [kg]
R = 0.05;   % wheel radius [m]
rou = 0.15; % wheel to center = wheel to wheel length [m]
J0 = 0.032; % body's nominal inertial [kgm^2]
J1 = 0.000625;  % nominal inertial of wheel [kgm^2]
J2 = 3.13*10^(-4);  %vertical axis passing through the wheel center mass inertia [kgm^2]

Jc = J0+4*(J2+m1*(2*rou^2));
ms = m0 + 4*M1;
A = ms*R^2/8 + Jc*R^2/(16*4*rou^2) + J1;
B = Jc*R^2/(16*4*rou^2);
C = ms*R^2/8 - Jc*R^2/(16*4*rou^2);

A1 = (A*(A-C)-2*B^2)/((A+C)*(A-2*B-C)*(A+2*B-C));
B1 = B/((A-2*B-C)*(A_2*B-C));
C1 = (C*(A-C)+2*B^2)/((A+C)*(A-2*B-C)*(A+2*B-C));

psi2dot2 = A1*M1 + B1*(M2-M3)-C1*M4;
psi2dot1 = A1*M2 + B1*(M1-M4)-C1*M3;
psi2dot3 = A1*M3 + B1*(M4-M1)-C1*M2;
psi2dot4 = A1*M4 + B1*(M3-M2)-C1*M1;

% global coordinates
yaw = R*(-psi2+psi1-psi3+psi4)/(8*rou) + constant;

a1 = R*cos(yaw-pi/4)/(2*sqrt(2)); a2 = R*cos(yaw+pi/4)/(2*sqrt(2));
b1 = R*cos(yaw+pi/4)/(2*sqrt(2)); b2 = R*cos(yaw-pi/4)/(2*sqrt(2));

xdot = a2*psidot2 + a1*psidot1 + a2*psidot3 + a1*psidot4;
ydot = b2*psidot2 + b1*psidot1 + b2*psidot3 + a1*psidot4;

function global_state = mecanum4wheelDynamics(thdot, th2dot, g_psi)
% th = [theta1 theta2 theta3 theta4]^T
% global_state = [x2dot y2dot psi2dot]^T

% 0 : body frame, 1: wheel fixed frame
m0 = 3.1;   % body mass [kg]
m1 = 0.35;  % wheel mass [kg]
R = 0.05;   % wheel radius [m]
rou = 0.15; % wheel to center = wheel to wheel length [m]
J0 = 0.032; % body's nominal inertial [kgm^2]
J1 = 0.000625;  % nominal inertial of wheel [kgm^2]
b1 = 0.087;  % friction of wheel [Nms/rad]
H = [0 J1/2 0 J1/2;
     -J1/2 0 -J1/2 0;
     0 J1/2 0 J1/2;
     -J1/2 0 -J1/2 0];
 
hpsi_mat = h_psi(g_psi);
% global position Q = [x y psi]^T
global_state = R/(4*J1) * hpsi_mat * (H*thdot + J1*th2dot);
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

