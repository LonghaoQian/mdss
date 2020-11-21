%% load the 

close all

loadloggeddata('datalog');
load('datalog.mat')

%%
figure(1)
subplot(3,2,1)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('omega_dotx')))
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(Omega_dot(1,:,:),2001,1))
legend('solver','simulink')
xlabel('t(s)')
ylabel('omega_dotx')
grid on
title('angular acc x')

subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(Omega_dot(1,:,:),2001,1) - logged_data.data(:,logged_data.tagmap('omega_dotx')))
xlabel('t(s)')
ylabel('omega_dotx')
grid on
title('angular acc x diff')

subplot(3,2,3)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('omega_doty')))
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(Omega_dot(2,:,:),2001,1))
legend('solver','simulink')
xlabel('t(s)')
ylabel('omega_doty')
grid on
title('angular acc y')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(Omega_dot(2,:,:),2001,1) - logged_data.data(:,logged_data.tagmap('omega_doty')))
xlabel('t(s)')
ylabel('omega_doty')
grid on
title('angular acc y diff')

subplot(3,2,5)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('omega_dotz')))
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(Omega_dot(3,:,:),2001,1))
legend('solver','simulink')
xlabel('t(s)')
ylabel('omega_dotz')
grid on
title('angular acc z')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(Omega_dot(3,:,:),2001,1) - logged_data.data(:,logged_data.tagmap('omega_dotz')))
xlabel('t(s)')
ylabel('omega_dotz')
grid on
title('angular acc z diff')
%%
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
%%
figure(6)
subplot(3,2,1)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('Vbx')))
plot(logged_data.data(:,logged_data.tagmap('t')),VB(:,1))
legend('solver','simulink')
xlabel('t(s)')
ylabel('Vbx(m/s)')
grid on
title('Vbx')

subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),VB(:,1) - logged_data.data(:,logged_data.tagmap('Vbx')))
xlabel('t(s)')
ylabel('Vbx(m/s)')
grid on
title('Vbx diff')

subplot(3,2,3)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('Vby')))
plot(logged_data.data(:,logged_data.tagmap('t')),VB(:,2))
legend('solver','simulink')
xlabel('t(s)')
ylabel('Vby(m/s)')
grid on
title('Vby')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),VB(:,2) - logged_data.data(:,logged_data.tagmap('Vby')))
xlabel('t(s)')
ylabel('Vby(m/s)')
grid on
title('Vby diff')

subplot(3,2,5)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('Vbz')))
plot(logged_data.data(:,logged_data.tagmap('t')),VB(:,3))
legend('solver','simulink')
xlabel('t(s)')
ylabel('Vbz(m/s)')
grid on
title('Vbz')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),VB(:,3)- logged_data.data(:,logged_data.tagmap('Vbz')))
xlabel('t(s)')
ylabel('Vbz')
grid on
title('Vbz diff')
%%
figure(7)
subplot(3,2,1)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('XIx')))
plot(logged_data.data(:,logged_data.tagmap('t')),XI(:,1))
legend('solver','simulink')
xlabel('t(s)')
ylabel('XIx(m)')
grid on
title('XIx')

subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),XI(:,1) - logged_data.data(:,logged_data.tagmap('XIx')))
xlabel('t(s)')
ylabel('XIx(m)')
grid on
title('XIx diff')

subplot(3,2,3)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('XIy')))
plot(logged_data.data(:,logged_data.tagmap('t')),XI(:,2))
legend('solver','simulink')
xlabel('t(s)')
ylabel('XIy(m)')
grid on
title('XIy')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),XI(:,2) - logged_data.data(:,logged_data.tagmap('XIy')))
xlabel('t(s)')
ylabel('Vby(m/s)')
grid on
title('XIy diff')

subplot(3,2,5)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('XIz')))
plot(logged_data.data(:,logged_data.tagmap('t')),XI(:,3))
legend('solver','simulink')
xlabel('t(s)')
ylabel('XIz(m/s)')
grid on
title('XIz')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),XI(:,3) - logged_data.data(:,logged_data.tagmap('XIz')))
xlabel('t(s)')
ylabel('XIz')
grid on
title('XIz diff')
%% 
figure(8)
subplot(3,2,1)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('Vb_dotx')))
plot(logged_data.data(:,logged_data.tagmap('t')),Vb_dot(:,1))
legend('solver','simulink')
xlabel('t(s)')
ylabel('Vb_dotx(m/s^2)')
grid on
title('Vb_dotx')

subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),Vb_dot(:,1) - logged_data.data(:,logged_data.tagmap('Vb_dotx')))
xlabel('t(s)')
ylabel('Vb_dotx(m/s^2)')
grid on
title('Vb_dotx diff')

subplot(3,2,3)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('Vb_doty')))
plot(logged_data.data(:,logged_data.tagmap('t')),Vb_dot(:,2))
legend('solver','simulink')
xlabel('t(s)')
ylabel('Vb_doty(m/s^2)')
grid on
title('Vb_doty')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),Vb_dot(:,2) - logged_data.data(:,logged_data.tagmap('Vb_doty')))
xlabel('t(s)')
ylabel('Vb_doty(m/s^2)')
grid on
title('Vb_doty diff')

subplot(3,2,5)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('Vb_dotz')))
plot(logged_data.data(:,logged_data.tagmap('t')),Vb_dot(:,3))
legend('solver','simulink')
xlabel('t(s)')
ylabel('Vb_dotz(m/s^2)')
grid on
title('Vb_dotz')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),Vb_dot(:,3) - logged_data.data(:,logged_data.tagmap('Vb_dotz')))
xlabel('t(s)')
ylabel('Vb_dotz')
grid on
title('Vb_dotz diff')
%%
figure(9)
subplot(3,2,1)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('AOA')))
plot(logged_data.data(:,logged_data.tagmap('t')),AOA(:,1))
legend('solver','simulink')
xlabel('t(s)')
ylabel('AOA(rad)')
grid on
title('AOA')

subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),AOA(:,1) - logged_data.data(:,logged_data.tagmap('AOA')))
xlabel('t(s)')
ylabel('AOA(rad)')
grid on
title('AOA')

subplot(3,2,3)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('Sideslip')))
plot(logged_data.data(:,logged_data.tagmap('t')),sideslip(:,1))
legend('solver','simulink')
xlabel('t(s)')
ylabel('Sideslip(rad)')
grid on
title('Sideslip')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),sideslip(:,1) - logged_data.data(:,logged_data.tagmap('Sideslip')))
xlabel('t(s)')
ylabel('Sideslip(rad)')
grid on
title('Sideslip diff')

subplot(3,2,5)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('TAS')))
plot(logged_data.data(:,logged_data.tagmap('t')),TAS(:,1))
legend('solver','simulink')
xlabel('t(s)')
ylabel('TAS(m/s)')
grid on
title('TAS')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),TAS(:,1) - logged_data.data(:,logged_data.tagmap('TAS')))
xlabel('t(s)')
ylabel('TAS(m/s)')
grid on
title('TAS diff')
%%
figure(10)
subplot(4,2,1)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('AOArate')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),AOArate(:,1))
legend('solver','simulink')
xlabel('t(s)')
ylabel('AOArate(rad)')
grid on
title('AOArate')

subplot(4,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),AOArate(:,1) - logged_data.data(:,logged_data.tagmap('AOArate')))
xlabel('t(s)')
ylabel('AOArate(rad/s)')
grid on
title('AOArate')

subplot(4,2,3)
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('Sidesliprate')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),beta_dotbar(:,1))
legend('solver','simulink')
xlabel('t(s)')
ylabel('Sidesliprate(rad/s)')
grid on
title('Sidesliprate')

subplot(4,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),beta_dotbar(:,1) - logged_data.data(:,logged_data.tagmap('Sidesliprate')))
xlabel('t(s)')
ylabel('beta_dotbar(rad/s)')
grid on
title('beta_dotbar')

subplot(4,2,5)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('dynamicpressure')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),dynamicpressure(:,1))
legend('solver','simulink')
xlabel('t(s)')
ylabel('dynamicpressure(PA)')
grid on
title('dynamicpressure')

subplot(4,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),dynamicpressure(:,1) - logged_data.data(:,logged_data.tagmap('dynamicpressure')))
xlabel('t(s)')
ylabel('dynamicpressure(PA)')
grid on
title('dynamicpressure')

subplot(4,2,7)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('gamma')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),gamma)
legend('solver','simulink')
xlabel('t(s)')
ylabel('gamma(rad)')
grid on
title('gamma')

subplot(4,2,8)
plot(logged_data.data(:,logged_data.tagmap('t')),gamma - logged_data.data(:,logged_data.tagmap('gamma')))
xlabel('t(s)')
ylabel('gamma(rad)')
grid on
title('gamma')

%%
figure(11)
subplot(3,2,1)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('Pbar')))
plot(logged_data.data(:,logged_data.tagmap('t')),p_q_rbar(:,1))
legend('solver','simulink')
xlabel('t(s)')
ylabel('Pbar(rad/s)')
grid on
title('Pbar')

subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),p_q_rbar(:,1) - logged_data.data(:,logged_data.tagmap('Pbar')))
xlabel('t(s)')
ylabel('Pbar(rad/s)')
grid on
title('Pbar diff')

subplot(3,2,3)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('Qbar')))
plot(logged_data.data(:,logged_data.tagmap('t')),p_q_rbar(:,2))
legend('solver','simulink')
xlabel('t(s)')
ylabel('Qbar(rad/s)')
grid on
title('Qbar')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),p_q_rbar(:,2) - logged_data.data(:,logged_data.tagmap('Qbar')))
xlabel('t(s)')
ylabel('Qbar(rad/s)')
grid on
title('Qbar diff')

subplot(3,2,5)
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap('Rbar')))
plot(logged_data.data(:,logged_data.tagmap('t')),p_q_rbar(:,3))
legend('solver','simulink')
xlabel('t(s)')
ylabel('Rbar(rad/s)')
grid on
title('Rbar')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),p_q_rbar(:,3) - logged_data.data(:,logged_data.tagmap('Rbar')))
xlabel('t(s)')
ylabel('Rbar(rad/s)')
grid on
title('Rbar diff')
%% 
figure(12)

subplot(3,3,1)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(R_WB(1,1,:),2001,1) - logged_data.data(:,logged_data.tagmap('RWB00')))
xlabel('t(s)')
ylabel('RWB00')
grid on
title('RWB00 diff')

subplot(3,3,2)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(R_WB(1,2,:),2001,1) - logged_data.data(:,logged_data.tagmap('RWB01')))
xlabel('t(s)')
ylabel('RWB01')
grid on
title('RWB01 diff')

subplot(3,3,3)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(R_WB(1,3,:),2001,1) - logged_data.data(:,logged_data.tagmap('RWB02')))
xlabel('t(s)')
ylabel('RWB02')
grid on
title('RWB02 diff')

subplot(3,3,4)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(R_WB(2,1,:),2001,1) - logged_data.data(:,logged_data.tagmap('RWB10')))
xlabel('t(s)')
ylabel('RWB10')
grid on
title('RWB10 diff')

subplot(3,3,5)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(R_WB(2,2,:),2001,1) - logged_data.data(:,logged_data.tagmap('RWB11')))
xlabel('t(s)')
ylabel('RWB11')
grid on
title('RWB11 diff')

subplot(3,3,6)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(R_WB(2,3,:),2001,1) - logged_data.data(:,logged_data.tagmap('RWB12')))
xlabel('t(s)')
ylabel('RWB02')
grid on
title('RWB12 diff')

subplot(3,3,7)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(R_WB(3,1,:),2001,1) - logged_data.data(:,logged_data.tagmap('RWB20')))
xlabel('t(s)')
ylabel('RWB00')
grid on
title('RWB20 diff')

subplot(3,3,8)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(R_WB(3,2,:),2001,1) - logged_data.data(:,logged_data.tagmap('RWB21')))
xlabel('t(s)')
ylabel('RWB21')
grid on
title('RWB21 diff')

subplot(3,3,9)
plot(logged_data.data(:,logged_data.tagmap('t')),reshape(R_WB(3,3,:),2001,1) - logged_data.data(:,logged_data.tagmap('RWB22')))
xlabel('t(s)')
ylabel('RWB22')
grid on
title('RWB22 diff')
%%
figure(13)
subplot(3,2,1)
plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('FBx')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),AFx)
legend('solver','simulink')
xlabel('t(s)')
ylabel('FBx(N)')
grid on
title('FBx')

subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),AFx - logged_data.data(:,logged_data.tagmap('FBx')))
xlabel('t(s)')
ylabel('FBx(N)')
grid on
title('FBx diff')

subplot(3,2,3)
plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('FBy')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),AFy)
legend('solver','simulink')
xlabel('t(s)')
ylabel('FBy(N)')
grid on
title('FBy')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),AFy - logged_data.data(:,logged_data.tagmap('FBy')))
xlabel('t(s)')
ylabel('FBy(N)')
grid on
title('FBy diff')

subplot(3,2,5)
plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('FBz')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),AFz)
legend('solver','simulink')
xlabel('t(s)')
ylabel('FBz(N)')
grid on
title('FBz')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),AFz - logged_data.data(:,logged_data.tagmap('FBz')))
xlabel('t(s)')
ylabel('FBz(N)')
grid on
title('FBz diff')
%%
figure(14)
subplot(3,2,1)
plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('MBx')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),ATx)
legend('solver','simulink')
xlabel('t(s)')
ylabel('MBx(Nm)')
grid on
title('MBx')

subplot(3,2,2)
plot(logged_data.data(:,logged_data.tagmap('t')),ATx - logged_data.data(:,logged_data.tagmap('MBx')))
xlabel('t(s)')
ylabel('MBx(Nm)')
grid on
title('MBx diff')

subplot(3,2,3)
plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('MBy')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),ATy)
legend('solver','simulink')
xlabel('t(s)')
ylabel('MBy(Nm)')
grid on
title('FBy')

subplot(3,2,4)
plot(logged_data.data(:,logged_data.tagmap('t')),ATy - logged_data.data(:,logged_data.tagmap('MBy')))
xlabel('t(s)')
ylabel('MBy(Nm)')
grid on
title('MBy diff')

subplot(3,2,5)
plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('MBz')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),ATz)
legend('solver','simulink')
xlabel('t(s)')
ylabel('MBz(Nm)')
grid on
title('MBz')

subplot(3,2,6)
plot(logged_data.data(:,logged_data.tagmap('t')),ATz - logged_data.data(:,logged_data.tagmap('MBz')))
xlabel('t(s)')
ylabel('MBz(Nm)')
grid on
title('MBz diff')
%%
figure(15)
subplot(4,2,1)

plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('Thrust')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')), Thrust)
legend('solver','simulink')
xlabel('t(s)')
ylabel('Thrust(N)')
grid on
title('Thrust')


subplot(4,2,2)

plot(logged_data.data(:,logged_data.tagmap('t')),Thrust - logged_data.data(:,logged_data.tagmap('Thrust')))

xlabel('t(s)')
ylabel('Thrust (N)')
grid on
title('Thrust diff')

subplot(4,2,3)

plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('CT')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')), CT)
legend('solver','simulink')
xlabel('t(s)')
ylabel('CT')
grid on
title('CT')


subplot(4,2,4)

plot(logged_data.data(:,logged_data.tagmap('t')),CT - logged_data.data(:,logged_data.tagmap('CT')))

xlabel('t(s)')
ylabel('CT')
grid on
title('CT diff')


subplot(4,2,5)

plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('TorqueRequired')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')), TorqueRequired)
legend('solver','simulink')
xlabel('t(s)')
ylabel('TorqueRequired(W/rps)')
grid on
title('TorqueRequired')


subplot(4,2,6)

plot(logged_data.data(:,logged_data.tagmap('t')),TorqueRequired - logged_data.data(:,logged_data.tagmap('TorqueRequired')))

xlabel('t(s)')
ylabel('TorqueRequired (Nm)')
grid on
title('TorqueRequired diff')

subplot(4,2,7)

plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('CP')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')), CP)
legend('solver','simulink')
xlabel('t(s)')
ylabel('CP')
grid on

subplot(4,2,8)

plot(logged_data.data(:,logged_data.tagmap('t')),CP - logged_data.data(:,logged_data.tagmap('CP')))

xlabel('t(s)')
ylabel('CP')
grid on
title('CP diff')
%%
figure(16)

subplot(2,2,1)

plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('TorqueAvaliable')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')),TorqueAvaliable)
legend('solver','simulink')
xlabel('t(s)')
ylabel('TorqueAvaliable(Nm)')
grid on


subplot(2,2,2)

plot(logged_data.data(:,logged_data.tagmap('t')),TorqueAvaliable - logged_data.data(:,logged_data.tagmap('TorqueAvaliable')))
% 
xlabel('t(s)')
ylabel('TorqueAvaliable (Nm)')
grid on
title('TorqueAvaliable diff')

subplot(2,2,3)

plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('Shaftrps')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')), sharftprs)
legend('solver','simulink')
xlabel('t(s)')
ylabel('sharftprs(RPS)')
grid on


subplot(2,2,4)

plot(logged_data.data(:,logged_data.tagmap('t')),sharftprs - logged_data.data(:,logged_data.tagmap('Shaftrps')))
% 
xlabel('t(s)')
ylabel('sharftprs(Nm)')
grid on
title('sharftprs diff')
%%

figure(17)
subplot(3,2,1)

plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('phidot')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')), phi_dot)
legend('solver','simulink')
xlabel('t(s)')
ylabel('phi_dot(rad/s)')
grid on
title('phi_dot')

subplot(3,2,2)

plot(logged_data.data(:,logged_data.tagmap('t')), phi_dot - logged_data.data(:,logged_data.tagmap('phidot')))
% 
xlabel('t(s)')
ylabel('phi_dot(rad/s)')
grid on
title('phi_dot diff')

subplot(3,2,3)

plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('thetadot')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')), theta_dot)
legend('solver','simulink')
xlabel('t(s)')
ylabel('thetadot(rad/s)')
grid on
title('thetadot')

subplot(3,2,4)

plot(logged_data.data(:,logged_data.tagmap('t')),theta_dot - logged_data.data(:,logged_data.tagmap('thetadot')))
% 
xlabel('t(s)')
ylabel('thetadot(rad/s)')
grid on
title('thetadot diff')

subplot(3,2,5)

plot(logged_data.data(:,logged_data.tagmap('t')), phi_dot - logged_data.data(:,logged_data.tagmap('psidot')))
% 
xlabel('t(s)')
ylabel('phi_dot(rad/s)')
grid on
title('phi_dot diff')

plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('psidot')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')), psi_dot)
legend('solver','simulink')
xlabel('t(s)')
ylabel('psi_dott(rad/s)')
grid on
title('psi_dot')

subplot(3,2,6)

plot(logged_data.data(:,logged_data.tagmap('t')),psi_dot - logged_data.data(:,logged_data.tagmap('psidot')))
% 
xlabel('t(s)')
ylabel('psi_dot(rad/s)')
grid on
title('psi_dot diff')
%%
figure(18)
subplot(3,2,1)

plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('Snose')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')), S0)
legend('solver','simulink')
xlabel('t(s)')
ylabel('compression (m)')
grid on
title('compression nose gear')

subplot(3,2,2)

plot(logged_data.data(:,logged_data.tagmap('t')), S0 - logged_data.data(:,logged_data.tagmap('Snose')))
% 
xlabel('t(s)')
ylabel('compression (m)')
grid on
title('compression nose gear diff')


subplot(3,2,3)

plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('Sleft')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')), S1)
legend('solver','simulink')
xlabel('t(s)')
ylabel('compression (m)')
grid on
title('compression left gear')

subplot(3,2,4)

plot(logged_data.data(:,logged_data.tagmap('t')), S1 - logged_data.data(:,logged_data.tagmap('Sleft')))
% 
xlabel('t(s)')
ylabel('compression (m)')
grid on
title('compression left gear diff')

subplot(3,2,5)

plot(logged_data.data(:,logged_data.tagmap('t')), logged_data.data(:,logged_data.tagmap('Sright')))
hold on
plot(logged_data.data(:,logged_data.tagmap('t')), S2)
legend('solver','simulink')
xlabel('t(s)')
ylabel('compression (m)')
grid on
title('compression right gear')

subplot(3,2,6)

plot(logged_data.data(:,logged_data.tagmap('t')), S2 - logged_data.data(:,logged_data.tagmap('Sright')))
% 
xlabel('t(s)')
ylabel('compression (m)')
grid on
title('compression right gear diff')
%%


