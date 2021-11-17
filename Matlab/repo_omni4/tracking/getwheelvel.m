% wheel velocity calculator (rad/sec), dependent on current mobile robot's
% state
function wheelVel= getwheelvel(psi, cur_state_dot)
R = 0.05;   % wheel radius [m]
hpsi_inv_mat = hpsi_inv(psi);

wheelVel = 4/R* hpsi_inv_mat * cur_state_dot;
end