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