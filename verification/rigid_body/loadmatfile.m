clear all
load('rigid_body_state.mat')
% construct input
F_B = [A(:,1) A(:,23:25)];
M_B = [A(:,1) A(:,26:28)];
X_I = A(1,8:10);
Vb_I = A(1,20:22);
Euler_I = [0 0 0];
Omega_I = A(1,5:7);