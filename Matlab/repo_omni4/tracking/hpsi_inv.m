% matrix dependent on mobile robot's yaw angle
function mat = hpsi_inv(psi)
psi = psi + pi/4;
sqrt2 = sqrt(2);
mat = 0.25 * [-sqrt2*sin(psi) sqrt2*cos(psi) 0.3;
                sqrt2*cos(psi) sqrt2*sin(psi) -0.3;
                -sqrt2*sin(psi) sqrt2*cos(psi) -0.3;
                sqrt2*cos(psi) sqrt2*sin(psi) 0.3];
end