% analysis
t_s = A(:,1);
figure(1)
subplot(3,1,1)
plot(t_s,A(:,20)-V_b(:,1))
subplot(3,1,2)
plot(t_s,A(:,21)-V_b(:,2))
subplot(3,1,3)
plot(t_s,A(:,22)-V_b(:,3))

figure(2)
subplot(3,1,1)
plot(t_s,A(:,2)-V_I(:,1))
subplot(3,1,2)
plot(t_s,A(:,3)-V_I(:,2))
subplot(3,1,3)
plot(t_s,A(:,4)-V_I(:,3))

figure(3)
subplot(3,1,1)
plot(t_s,A(:,5)-omega(:,1))
subplot(3,1,2)
plot(t_s,A(:,6)-omega(:,2))
subplot(3,1,3)
plot(t_s,A(:,7)-omega(:,3))

figure(4)
subplot(3,1,1)
plot(t_s,A(:,8)-X_I(:,1))
subplot(3,1,2)
plot(t_s,A(:,9)-X_I(:,2))
subplot(3,1,3)
plot(t_s,A(:,10)-X_I(:,3))

Euler_A = zeros(size(DCM));% R_BI
for i =1: max(size(A))
    R = [A(i,11:13)' A(i,14:16)' A(i,17:19)'];% R_IB
    Euler_A(:,:,i)= R*reshape(DCM(:,:,i),3,3);
end


