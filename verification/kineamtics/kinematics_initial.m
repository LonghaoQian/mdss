% load the initial condition
omega_0 = [0;0;0];
euler0 = [0.0; 4.0/57.3; 0/57.3];
quaternion0  = angle2quat(euler0(3), euler0(2), euler0(1));
VI0 = [40;0;0];
XI0 = [0;0;-7];
% m = 700;
% J = diag([1285.31541660000, 1824.93096070000, 2666.89390765000]);
m = 1454.0*0.453592;% Kg
J = [948.0 0 0;
          0, 1346.0 0;
          0 ,0 1967.0]*1.35581795;%SLUG*FT2 - > kg m^2
J_inv = inv(J);
cbar = 1.493520000000000;
b = 10.097;
load('C172flightdata.mat');

propellerchart = [
0.0    ,0.0990   , 0.0400;
0.1000 ,  0.0950 ,   0.0406;
0.2000,  0.0880  ,  0.0406;
0.3000,  0.0780  ,  0.0400;
0.4000,    0.0645,    0.0366;
0.5000,    0.0495,    0.0318;
0.6000,    0.0340,    0.0250;
0.7000,    0.0185,    0.0160;
0.8000,    0.0040,    0.0050;
0.9000, - 0.0160, - 0.0067;
1.0000, - 0.0300, - 0.0150;
1.1000, - 0.0400, - 0.0200;
1.2000, - 0.0500, - 0.0250;
1.5000, - 0.0550, - 0.0270;
1.6000, - 0.0650, - 0.0300;
2.0000, - 0.0750, - 0.0330];


pistonengine_torque = C172.Engine.BHP.Table./(2*pi*C172.Engine.BHP.RowRPM'/60);

%% gear position
H_min = 10;%m

gearposition0 =  [1.2141;0; 1.4351];
geardirection0 = [0 0 1]';
stiffness0 = 1800*4.4482/0.3048; % LBS/FT
damping0 = 500*4.4482/0.3048; % LBS/FT/SEC
damping_rebound0 = 2000*4.4482/0.3048; % LBS/FT/SEC


gearposition1 = [-0.4369;-1.2763;1.3960];
geardirection1 = [0 0 1]';
stiffness1 = 5400*4.4482/0.3048;% LBS/FT
damping1 = 160*4.4482/0.3048;% LBS/FT/SEC
damping_rebound1 =  320*4.4482/0.3048;% LBS/FT/SEC

gearposition2 =   [ -0.4369;1.2763;1.3960];
geardirection2 = [0 0 1]';
stiffness2 = 5400*4.4482/0.3048;% LBS/FT
damping2 = 160*4.4482/0.3048;% LBS/FT/SEC
damping_rebound2 =  320*4.4482/0.3048;% LBS/FT/SEC


