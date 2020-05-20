% step 1 input system variables and establish statespace system
% servo = tf([10],[1 10]);
A = [-3.8916 * 10^-2 18.9920 -32.139 0.0000;
    -1.0285 * 10^-3  -0.64537 5.62290*10^-3 1.0000;
    0.0000 0.0000 0.0000 1.0000;
    8.08470*10^-5 -0.77287 -8.0979*10^-4 -0.52900];
B = [0;0;0;0.010992*57.3];

C= eye(size(A));

D = zeros(size(B));

A_bar = [A B;
         zeros(1,4) -10];
B_bar = [zeros(4,1);10];

K = [0 0 0 -1 0];

SYS = ss(A_bar+B_bar*K,B_bar,eye(size(A_bar)),zeros(size(B_bar)),'StateName',{'v' 'alpha','theta','q','de_pos'},...
                                         'InputName','de_cmd',...
                                         'OutputName',{'v' 'alpha','theta','q','de_pos'});
% step 2 obtain the transfer function from delta_e to theta
plant = tf(SYS);
SAS_loop = plant(3);
figure
bode(SAS_loop)
grid on
figure
closed_loop_0 = feedback(SAS_loop,1);
step(closed_loop_0)
grid on
% step 3 add a unit K feedback to the q channel and plot the bode plot
PI_compensator = 40 * tf([1 0.1],[1 0]);
Lead_compensator = tf([1 1.4],[1 14]);
% step 4 add the compensator and bode plot
closed_loop = feedback(PI_compensator*Lead_compensator*SAS_loop,1);
figure
bode(PI_compensator*Lead_compensator*SAS_loop)
grid on
figure
step(closed_loop,5)
grid on