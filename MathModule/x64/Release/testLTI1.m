% 
A =[ -0.2, 0.1, -1;
		0, -1, 0;
		0.1 0, -1];
B =[ 2, 2, 3]';
C =[ 1, 0, 0;
	0, 1, 0;
	0, 0, 1];
D =[ 0,0,0]';

x0 = [0 0 0]';


A1 = [0 ,- 0.1;
		1,   0];
B1 = [0;1];

C1 = [1, 0;
	0, 1];
D1 = [0, 0]';

x1 = [0,1]';

sim('LTI_test1.slx',20);

%sys = ss(A,B,C,D);

%//[Y,T] = step(sys,10);

close all
filename = 'LTIsimulation.txt';
fileID = fopen(filename,'r');
tline = fgetl(fileID);
data_raw = textscan(fileID,'%fXX%fXX%fXX%fXX%f');
fclose(fileID);
N = max(size(data_raw{1,1}));  % number of invalide data points
time = zeros(N,1);
X1 = zeros(N,1);
X2 = zeros(N,1);
X3 = zeros(N,1);
for i = 1:N
    time(i) = data_raw{1,2}(i);
    X1(i) = data_raw{1,3}(i);
    X2(i) = data_raw{1,4}(i);
    X3(i) = data_raw{1,5}(i);
end

figure(1)
subplot(3,1,1)
plot(time,X1,'-xk')
ylabel('Y1_1')
xlabel('time(s)')
hold on
plot(Y1.Time,Y1.Data(:,1),'-og')
legend('Simulink result','C++ solver result')
subplot(3,1,2)
plot(time,X2,'-xk')
ylabel('Y1_2')
xlabel('time(s)')
hold on
plot(Y1.Time,Y1.Data(:,2),'-og')
legend('Simulink result','C++ solver result')
subplot(3,1,3)
plot(time,X3,'-xk')
hold on
plot(Y1.Time,Y1.Data(:,3),'-og')
ylabel('Y1_3')
xlabel('time(s)')
legend('Simulink result','C++ solver result')



