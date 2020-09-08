clear all
load('linear_system_state.mat')
% construct input
input_solver = Data(:,2);
v_solver = Data(:,3);
alpha_solver = Data(:,4);
theta_solver = Data(:,5);
q_solver = Data(:,6);
sum_solver = Data(:,7);
A = [-3.8916 * 10^-2 18.9920 -32.139 0.0000;
    -1.0285 * 10^-3  -0.64537 5.62290*10^-3 1.0000;
    0.0000 0.0000 0.0000 1.0000;
    8.08470*10^-5 -0.77287 -8.0979*10^-4 -0.52900];
B = [0;0;0;0.010992*57.3];

C= eye(size(A));

D = zeros(size(B));