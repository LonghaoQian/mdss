%% load the 

close all

loadloggeddata('datalog');
load('datalog.mat')

%%
figure(1)
subplot(3,2,1)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('omega_dotx')))
plot(logged_data.data(:,logged_data.tagmap('t')),Omega_dot(:,1))
legend('solver','simulink')
xlabel('t(s)')
ylabel('omega_dotx')
grid on
title('angular acc x')

subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),Omega_dot(:,1) - logged_data.data(:,logged_data.tagmap('omega_dotx')))
xlabel('t(s)')
ylabel('omega_dotx')
grid on
title('angular acc x diff')

subplot(3,2,3)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('omega_doty')))
plot(logged_data.data(:,logged_data.tagmap('t')),Omega_dot(:,2))
legend('solver','simulink')
xlabel('t(s)')
ylabel('omega_doty')
grid on
title('angular acc y')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),Omega_dot(:,2) - logged_data.data(:,logged_data.tagmap('omega_doty')))
xlabel('t(s)')
ylabel('omega_doty')
grid on
title('angular acc y diff')

subplot(3,2,5)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('omega_dotz')))
plot(logged_data.data(:,logged_data.tagmap('t')),Omega_dot(:,3))
legend('solver','simulink')
xlabel('t(s)')
ylabel('omega_dotz')
grid on
title('angular acc z')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),Omega_dot(:,3) - logged_data.data(:,logged_data.tagmap('omega_dotz')))
xlabel('t(s)')
ylabel('omega_dotz')
grid on
title('angular acc z diff')

figure(2)

subplot(3,2,1)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('roll')))
plot(logged_data.data(:,logged_data.tagmap('t')),Euler(:,1))
legend('solver','simulink')
xlabel('t(s)')
ylabel('roll (rad)')
grid on
title('roll')

subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),Euler(:,1) - logged_data.data(:,logged_data.tagmap('roll')))
xlabel('t(s)')
ylabel('roll')
grid on
title('roll diff')

subplot(3,2,3)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('pitch')))
plot(logged_data.data(:,logged_data.tagmap('t')),Euler(:,2))
legend('solver','simulink')
xlabel('t(s)')
ylabel('pitch (rad)')
grid on
title('pitch')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),Euler(:,2) - logged_data.data(:,logged_data.tagmap('pitch')))
xlabel('t(s)')
ylabel('pitch')
grid on
title('pitch diff')

subplot(3,2,5)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('yaw')))
plot(logged_data.data(:,logged_data.tagmap('t')),Euler(:,3))
legend('solver','simulink')
xlabel('t(s)')
ylabel('yaw (rad)')
grid on
title('yaw')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),Euler(:,3) - logged_data.data(:,logged_data.tagmap('yaw')))
xlabel('t(s)')
ylabel('yaw')
grid on
title('yaw diff')
%%
figure(3)

subplot(3,3,1)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(RIB(1,1,:),2001,1) - logged_data.data(:,logged_data.tagmap('RIB00')))
xlabel('t(s)')
ylabel('RIB00')
grid on
title('RIB00 diff')

subplot(3,3,2)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(RIB(1,2,:),2001,1) - logged_data.data(:,logged_data.tagmap('RIB01')))
xlabel('t(s)')
ylabel('RIB01')
grid on
title('RIB01 diff')

subplot(3,3,3)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(RIB(1,3,:),2001,1) - logged_data.data(:,logged_data.tagmap('RIB02')))
xlabel('t(s)')
ylabel('RIB02')
grid on
title('RIB02 diff')

subplot(3,3,4)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(RIB(2,1,:),2001,1) - logged_data.data(:,logged_data.tagmap('RIB10')))
xlabel('t(s)')
ylabel('RIB10')
grid on
title('RIB10 diff')

subplot(3,3,5)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(RIB(2,2,:),2001,1) - logged_data.data(:,logged_data.tagmap('RIB11')))
xlabel('t(s)')
ylabel('RIB11')
grid on
title('RIB11 diff')

subplot(3,3,6)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(RIB(2,3,:),2001,1) - logged_data.data(:,logged_data.tagmap('RIB12')))
xlabel('t(s)')
ylabel('RIB12')
grid on
title('RIB12 diff')

subplot(3,3,7)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(RIB(3,1,:),2001,1) - logged_data.data(:,logged_data.tagmap('RIB20')))
xlabel('t(s)')
ylabel('RIB20')
grid on
title('RIB20 diff')

subplot(3,3,8)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(RIB(3,2,:),2001,1) - logged_data.data(:,logged_data.tagmap('RIB21')))
xlabel('t(s)')
ylabel('RIB21')
grid on
title('RIB21 diff')

subplot(3,3,9)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(RIB(3,3,:),2001,1) - logged_data.data(:,logged_data.tagmap('RIB22')))
xlabel('t(s)')
ylabel('RIB22')
grid on
title('RIB22 diff')
%%
figure(4)
subplot(4,1,1)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('pressure')))
legend('solver')
xlabel('t(s)')
ylabel('pressure (10^5 Pa)')
grid on
title('pressure')


subplot(4,1,2)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('soundspeed')))
legend('solver')
xlabel('t(s)')
ylabel('soundspeed (m/s)')
grid on
title('soundspeed')

subplot(4,1,3)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('temperature')))
legend('solver')
xlabel('t(s)')
ylabel('temperature (K)')
grid on
title('temperature')

subplot(4,1,4)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('density')))
legend('solver')
xlabel('t(s)')
ylabel('density (kg/m^2)')
grid on
title('density')
%%
figure(5)
subplot(3,2,1)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('g0')))
hold on 
temp_g0 = reshape(gg(1,:,:),2001,1);
plot(logged_data.data(:,logged_data.tagmap('t')),temp_g0)
legend('solver','simulink')
xlabel('t(s)')
ylabel('g0')
grid on
title('g0')

subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('g0'))-temp_g0)
xlabel('t(s)')
ylabel('g0')
grid on
title('g0 diff')

subplot(3,2,3)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('g1')))
hold on
temp_g1 = reshape(gg(2,:,:),2001,1);
plot(logged_data.data(:,logged_data.tagmap('t')),temp_g1)
legend('solver','simulink')
xlabel('t(s)')
ylabel('g1')
grid on
title('g1')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('g1'))-temp_g1)
xlabel('t(s)')
ylabel('g0')
grid on
title('g1 diff')


subplot(3,2,5)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('g2')))
hold on
temp_g2 = reshape(gg(3,:,:),2001,1);
plot(logged_data.data(:,logged_data.tagmap('t')),temp_g2)
legend('solver','simulink')
xlabel('t(s)')
ylabel('g2')
grid on
title('g2')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('g2'))-temp_g2)
xlabel('t(s)')
ylabel('g0')
grid on
title('g2 diff')

