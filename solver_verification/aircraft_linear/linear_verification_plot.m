close all;
t_s = input.time;
figure(1)
plot(t_s,input.data - input_solver)
ylabel('error')
xlabel('t (s)')
grid on
figure(2)
subplot(2,1,1)
hold on
plot(t_s,v,t_s,v_solver)
hold off
grid on
ylabel(' v ')
xlabel('t (s)')
legend('simulink','solver')
subplot(2,1,2)
hold on
plot(t_s,v - v_solver)
grid on
ylabel(' v error ')
xlabel('t (s)')
grid on
figure(3)
subplot(2,1,1)
hold on
plot(t_s,alpha,t_s,alpha_solver)
hold off
grid on
ylabel(' alpha ')
xlabel('t (s)')
legend('simulink','solver')
grid on
subplot(2,1,2)
hold on
plot(t_s,alpha - alpha_solver)
hold off
grid on
ylabel(' alpha error')
xlabel('t (s)')
grid on
figure(4)
subplot(2,1,1)
hold on
plot(t_s,q,t_s,q_solver)
hold off
grid on
ylabel(' q ')
xlabel('t (s)')
legend('simulink','solver')
grid on
subplot(2,1,2)
hold on
plot(t_s,q - q_solver)
hold off
grid on
ylabel(' q error ')
xlabel('t (s)')
figure(5)
subplot(2,1,1)
hold on
plot(t_s,theta,t_s,theta_solver)
hold off
grid on
ylabel(' theta ')
xlabel('t (s)')
legend('simulink','solver')
subplot(2,1,2)
hold on
plot(t_s,theta - theta_solver)
hold off
grid on
ylabel(' theta error ')
xlabel('t (s)')
figure(6)
subplot(2,1,1)
hold on
plot(t_s,sum_,t_s,sum_solver)
hold off
grid on
ylabel(' sum ')
xlabel('t (s)')
legend('simulink','solver')
subplot(2,1,2)
hold on
plot(t_s,sum_ - sum_solver)
hold off
grid on
ylabel(' sum error ')
xlabel('t (s)')