%Explicit MPC for Iris 3DR Quadcopter for z,theta,phi and psi control
% close all

%Iris 3DR Parameters
g = 9.81;
m = 1.37;
lx = 0.13;
ly = 0.21;
KT = 15.67e-6;
KD = 2.55e-7;
Im = 2.52e-10;
Ixx = 0.0219;
Iyy = 0.0109;
Izz = 0.0306;
Ax = 0.25;
Ay = 0.25;
Az = 0.25;
wo = 463; %Angular velocity that compensate the drones weigth
Ts = 0.01;

%State Space Linearized
Ac = [0 1 0 0 0 0 0 0;
     0 -Az/m 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 1;
     0 0 0 0 0 0 0 0];
 Bc = [0 0 0 0;
      1/m 0 0 0;
      0 0 0 0;
      0 1/Ixx 0 0;
      0 0 0 0;
      0 0 1/Iyy 0;
      0 0 0 0;
      0 0 0 1/Izz];
Cc = [1 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0;
     0 0 0 0 1 0 0 0;
     0 0 0 0 0 0 1 0];

 %Discrete State Space
 [A,B]= c2d(Ac,Bc,Ts);
 C = Cc;
 
 Nstate = size(A,1);
 Ncontrol = size(B,2);
 Nout = size(C,1);
 Nref = Nout;
 
 
Nsim = 5000;
x = zeros(Nstate,Nsim);
y = zeros(Nout,Nsim);
u = y;
vet_index = zeros(1,Nsim);
erro = zeros(Nout,Nsim);
t = (0:(Nsim-1))*Ts;
cont_out = 0;
cont_reg = 0;
vetor_repeticao = zeros(1,Nsim);

for i = 1:Nsim
    if i < (Nsim/2)
        ref(:,i) = [1 0.3 0.2 -0.1]';
    else
        ref(:,i) = [5 -0.3 0.5 0.1]';
    end
end  
%%

for i = 1:Nsim
     index = 0;
    y(:,i) = C*x(:,i); 
    erro(:,i) = ref(:,i) - y(:,i); 
    cont_reg(i) = 0;
    for j = 1:size(Regions,1)
        A_CRi = Regions{j,1};
        b_CRi = Regions{j,2};
        flag = 0;       
        for k = 1:size(A_CRi,1)          
            if((A_CRi(k,:)*[x(:,i); ref(:,i)]) > b_CRi(k))
                flag = 1;
            end
        end
        if flag == 0
            cont_reg(i) = cont_reg(i) + 1;
            index = j;
        end
    end
    vet_index(i) = index;
    u_calc = Regions{index,3}*[x(:,i); ref(:,i)] + Regions{index,4};
% 
     %u_calc = test_toolbox_5_5([x(:,i); ref]);
%    u_calc = expmpc.evaluate([x(:,i); ref]);

    u(:,i) = u_calc(:,1);
    
%     u_calc = test_toolbox_10_10([x(:,i); ref ]);
%     u(i) = u_calc(1);
    x(:,i+1) = A*x(:,i) + B*u(:,i);% + 0.1*[rand(1)-rand(1) ; rand(1)-rand(1)];

end

%%
figure
subplot(Nout,1,1)
plot(t,y(1,:))
hold on
plot(t,ref(1,:),'-r')
legend('Altura','Referencia')

subplot(Nout,1,2)
plot(t,y(2,:))
hold on
plot(t,ref(2,:),'-r')
legend('Phi','Referencia')

subplot(Nout,1,3)
plot(t,y(3,:))
hold on
plot(t,ref(3,:),'-r')
legend('Theta','Referencia')

subplot(Nout,1,4)
plot(t,y(4,:))
hold on
plot(t,ref(4,:),'-r')
legend('Psi','Referencia')

figure
subplot(Ncontrol,1,1)
plot(t,u(1,:),'-b')
legend('Empuxo')

subplot(Ncontrol,1,2)
plot(t,u(2,:),'-r')
legend('Torque Phi')

subplot(Ncontrol,1,3)
plot(t,u(3,:),'-k')
legend('Torque Theta')

subplot(Ncontrol,1,4)
plot(t,u(4,:),'-m')
legend('Torque Psi')