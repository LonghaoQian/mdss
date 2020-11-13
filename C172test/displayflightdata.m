%% load the 

close all

loadloggeddata('c172log');
load('c172log.mat')
%%
figure(1)
subplot(3,2,1)
plot(logged_data.data(:,logged_data.tagmap('t')),57.3*logged_data.data(:,logged_data.tagmap('roll')))
xlabel('t(s)')
ylabel('Euler angles (deg)')
grid on
title('Roll angles')

subplot(3,2,3)
plot(logged_data.data(:,logged_data.tagmap('t')),57.3*logged_data.data(:,logged_data.tagmap('pitch')))
xlabel('t(s)')
ylabel('Euler angles (deg)')
grid on
title('Pitch angles')

subplot(3,2,5)
plot(logged_data.data(:,logged_data.tagmap('t')),57.3*logged_data.data(:,logged_data.tagmap('yaw')))
xlabel('t(s)')
ylabel('Euler angles (deg)')
grid on
title('Yaw angles')

subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),57.3*logged_data.data(:,logged_data.tagmap('omegax')))
xlabel('t(s)')
grid on
title('Roll angular velocity')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),57.3*logged_data.data(:,logged_data.tagmap('omegay')))
xlabel('t(s)')
grid on
title('Pitch angular velocity')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),57.3*logged_data.data(:,logged_data.tagmap('omegaz')))
xlabel('t(s)')
grid on
title('Yaw angular velocity')
%%
figure(2)

subplot(3,2,1)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('AIx')))
xlabel('t(s)')
ylabel('Linear Acc (m/s^2)')
grid on
title('Inertial Acc X')

subplot(3,2,3)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('AIy')))
xlabel('t(s)')
ylabel('Linear Acc (m/s^2)')
grid on
title('Inertial Acc Y')

subplot(3,2,5)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('AIz')))
xlabel('t(s)')
ylabel('Linear Acc (m/s^2)')
grid on
title('Inertial Acc Z')


subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('omega_dotx')))
xlabel('t(s)')
ylabel('Angular Acc (rad/s^2)')
grid on
title('Angular Acc X')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('omega_doty')))
xlabel('t(s)')
ylabel('Angular Acc (rad/s^2)')
grid on
title('Angular Acc Y')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('omega_dotz')))
xlabel('t(s)')
ylabel('Angular Acc (rad/s^2)')
grid on
title('Angular Acc Z')


%%
figure(4)

subplot(3,2,1)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('FBx')))
xlabel('t(s)')
ylabel('Total force (N)')
grid on
title('Total body force X')

subplot(3,2,3)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('FBy')))
xlabel('t(s)')
ylabel('Total force (N)')
grid on
title('Total aero body force Y')

subplot(3,2,5)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('FBz')))
xlabel('t(s)')
ylabel('Total force (N)')
grid on
title('Total aero body force Z')


subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('MBx')))
xlabel('t(s)')
ylabel('Total moment (Nm)')
grid on
title('Total  aero body moment X')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('MBy')))
xlabel('t(s)')
ylabel('Total moment (Nm)')
grid on
title('Total  aero body moment Y')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('MBz')))
xlabel('t(s)')
ylabel('Total aero moment (Nm)')
grid on
title('Total body moment Z')


figure(5)

subplot(3,2,1)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('VIx')))
xlabel('t(s)')
ylabel('Velocity (m/s)')
grid on
title('Intertial velocity X')

subplot(3,2,3)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('VIy')))
xlabel('t(s)')
ylabel('Velocity (m/s)')
grid on
title('Intertial velocity Y')

subplot(3,2,5)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('VIz')))
xlabel('t(s)')
ylabel('Velocity (m/s)')
grid on
title('Intertial velocity Z')


subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('XIx')))
xlabel('t(s)')
ylabel('Position (Nm)')
grid on
title('Interial position X')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('XIy')))
xlabel('t(s)')
ylabel('Position (Nm)')
grid on
title('Interial position Y')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('XIz')))
xlabel('t(s)')
ylabel('Position (Nm)')
grid on
title('Interial position Z')

figure(6)

subplot(3,2,1)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('TAS')))
xlabel('t(s)')
ylabel('Velocity (m/s)')
grid on
title('True airspeed')

subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),57.3*logged_data.data(:,logged_data.tagmap('AOA')))
xlabel('t(s)')
ylabel('Angle (deg)')
grid on
title('AOA')

subplot(3,2,3)
plot(logged_data.data(:,logged_data.tagmap('t')),57.3*logged_data.data(:,logged_data.tagmap('Sideslip')))
xlabel('t(s)')
ylabel('Angle (deg)')
grid on
title('Sideslip')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),57.3*logged_data.data(:,logged_data.tagmap('AOArate')))
xlabel('t(s)')
ylabel('Angle rate (deg/s)')
grid on
title('Sideslip')

subplot(3,2,5)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('dynamicpressure')))
xlabel('t(s)')
ylabel('Pressure (Pa)')
grid on
title('Dynamics pressure')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),57.3*logged_data.data(:,logged_data.tagmap('gamma')))
xlabel('t(s)')
ylabel('Angle (deg)')
grid on
title('Flight path angle')


figure(7)

subplot(3,2,1)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('Thrust')))
xlabel('t(s)')
ylabel('Force (N)')
grid on
title('Thrust')

subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('TorqueRequired')))
xlabel('t(s)')
ylabel('Torque (Nm)')
grid on
title('TorqueRequired')

subplot(3,2,3)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('CT')))
xlabel('t(s)')
ylabel('Coefficient ')
grid on
title('Thrust Coefficient')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('CP')))
xlabel('t(s)')
ylabel('Coefficient ')
grid on
title('Power Coefficient')

subplot(3,2,5)
plot(logged_data.data(:,logged_data.tagmap('t')),60*logged_data.data(:,logged_data.tagmap('Shaftrps')))
xlabel('t(s)')
ylabel('Angular rate (RPM)')
grid on
title('Shaft rpm')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('TorqueAvaliable')))
xlabel('t(s)')
ylabel('Torque (Nm)')
grid on
title('Torque avaliable')
%%
figure(8)

subplot(3,2,1)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('g0')))
xlabel('t(s)')
ylabel('Force (N)')
grid on
title('Gravity in body frame X')

subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('g1')))
xlabel('t(s)')
ylabel('Force (N)')
grid on
title('Gravity in body frame Y')

subplot(3,2,3)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('g2')))
xlabel('t(s)')
ylabel('Force (N)')
grid on
title('Gravity in body frame Z')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('TOTALX')))
xlabel('t(s)')
ylabel('Force (N)')
grid on
title('Total inerita force X')

subplot(3,2,5)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('TOTALY')))
xlabel('t(s)')
ylabel('Force (N)')
grid on
title('Total inerita force Y')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('TOTALZ')))
xlabel('t(s)')
ylabel('Force (N)')
grid on
title('Total inerita force Z')
%%
figure(9)

subplot(3,2,1)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('loadfactorbodyx')))
xlabel('t(s)')
ylabel('loadfactorbodyx')
grid on
title('loadfactorbodyx')

subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('loadfactorbodyy')))
xlabel('t(s)')
ylabel('loadfactorbodyy')
grid on
title('loadfactorbodyy')

subplot(3,2,3)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('loadfactorbodyz')))
xlabel('t(s)')
ylabel('loadfactorbodyz')
grid on
title('loadfactorbodyz')


subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('ThrottleCmd')))
xlabel('t(s)')
ylabel('ThrottleCmd')
grid on
title('ThrottleCmd')

subplot(3,2,5)
plot(logged_data.data(:,logged_data.tagmap('t')),196.85039370078738*logged_data.data(:,logged_data.tagmap('climbrate')))
xlabel('t(s)')
ylabel('climbrate (fpm)')
grid on
title('climbrate')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('height')))
xlabel('t(s)')
ylabel('height (m)')
grid on
title('height')

