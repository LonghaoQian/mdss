% load the initial condition
omega_0 = [0;0;0];
euler0 = [0.0; 5.0/57.3; 30.0/57.3];
quaternion0  = angle2quat(euler0(3), euler0(2), euler0(1));
VI0 = [40;0;0];
XI0 = zeros(3,1);
% m = 700;
% J = diag([1285.31541660000, 1824.93096070000, 2666.89390765000]);
m = 1;
J = diag([20 20 20]);
J_inv = inv(J);
cbar = 1.493520000000000;
b = 10.097;
