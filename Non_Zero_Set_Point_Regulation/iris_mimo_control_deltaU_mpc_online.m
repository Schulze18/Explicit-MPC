%MPC Online
clc

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
 
%Explicit MPC Parameters
Q = diag([3,1,1,1]);
Ru = 0.2*diag([0.001,0.01,0.01,0.01]);
Rdu = 0.5*diag([0.005,0.05,0.05,0.05]);
% % % Q = diag([0.7/10^2, 0.1/0.5^2, 0.1/0.5^2 0.1/0.5^2]);
% % % Ru = 0.01*diag([0.7/(2*m*g)^2, 0.1, 0.1, 0.1]);
% % % Rdu = 0.01*diag([0.7/(8*m)^2, 0.1/0.1^2, 0.1/0.1^2, 0.1/0.1^2]);

Ny = 5;
Nu = 5;

deltaU_max = [8*m Inf Inf Inf]';
deltaU_min = [-4*m -Inf -Inf -Inf]';
 
U_max = [2*m*g Inf Inf Inf]';
U_min = [-m*g -Inf -Inf -Inf];
 
Ref_max = [];%[30 Inf Inf Inf]';
Ref_min = [];%[-1 -Inf -Inf -Inf]';
 
X_max = [];%[Inf 1 2 Inf Inf Inf Inf Inf]';
X_min = [];%-[Inf 10 Inf Inf Inf Inf Inf Inf]';

Y_max = [];%[10 Inf Inf Inf]';
Y_min = [];%[-1.5 -Inf -Inf -Inf]';
 
% Code parameters
tol = 1e-7;
n_plot = 20;
last_plot = 0;
 
%%
[H, F, Sx, Su, Sdu, T_du, Qlinha, Rulinha, Rdulinha, Clinha] = setpoint_deltaU_matrices_cost_function(A, B, C, Q, Ru, Rdu, Nstate, Ncontrol, Nout, Ny, Nu);

[G, W, E, S, num_Gu] = setpoint_deltaU_generic_constraints(Sx, Su, Sdu, Clinha, H, F, Nstate, Ncontrol, Nout, Nref, Ny, Nu, deltaU_max, deltaU_min, U_max, U_min, Ref_max, Ref_min, X_max, X_min, Y_max, Y_min);

%%
Nsim = 5000;
x = zeros(Nstate,Nsim);
y = zeros(Nout,Nsim);
u = y;
deltau = y;

vet_index = zeros(1,Nsim);
erro = zeros(Nout,Nsim);
t = (0:(Nsim-1))*Ts;
cont_out = 0;
cont_reg = 0;
vetor_repeticao = zeros(1,Nsim);
ref = zeros(4,Nsim);

for i = 1:Nsim
    if i < (Nsim/2)
        ref(:,i) = [10 0 0 0]';
    else
        ref(:,i) = [10 0 0 0]';
    end
end  


options = optimoptions('quadprog','Display','None');
for i = 2:Nsim
    index = 0;
    y(:,i) = C*x(:,i); 
    erro(:,i) = ref(:,i) - y(:,i); 
    
    f_quad = F'*[x(:,i); u(:,i-1); ref(:,i)];
    H_quad = H;
    A_ineq = G;
    b_ineq = W + E*[x(:,i); u(:,i-1); ref(:,i)];
   
    delta_u_calc = quadprog(H_quad, f_quad, A_ineq, b_ineq,[],[],[],[],[],options);
    
    deltau(:,i) = double(delta_u_calc(1:Ncontrol,1));
    
    u(:,i) = u(:,i-1) + deltau(:,i);
    x(:,i+1) = A*x(:,i) + B*u(:,i);
    
end

%%
%%%%%Plot Saidas - Y
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

%%%%%Plot Ações de Controle - U
figure
subplot(Ncontrol,1,1)
plot(t,u(1,:),'-b')
hold on
if (isempty(U_max) == 0) 
    if(U_max(1) ~= Inf)
        plot(t,U_max(1)*ones(1,Nsim),'-r')
    end
end
if (isempty(U_min) == 0)
    if(U_min(1) ~= -Inf)
        plot(t,U_min(1)*ones(1,Nsim),'-r')
    end
end
title('U')
legend('Empuxo')

subplot(Ncontrol,1,2)
plot(t,u(2,:),'-g')
hold on
if (isempty(U_max) == 0)
    if(U_max(2) ~= Inf)
        plot(t,U_max(2)*ones(1,Nsim),'-r')
    end
end
if (isempty(U_max) == 0)
    if(U_min(2) ~= -Inf)
        plot(t,U_min(2)*ones(1,Nsim),'-r')
    end
end
legend('Torque Phi')

subplot(Ncontrol,1,3)
plot(t,u(3,:),'-k')
hold on
if (isempty(U_max) == 0)
    if(U_max(3) ~= Inf)
        plot(t,U_max(3)*ones(1,Nsim),'-r')
    end
end
if (isempty(U_max) == 0)
    if(U_min(3) ~= -Inf)
        plot(t,U_min(3)*ones(1,Nsim),'-r')
    end
end
legend('Torque Theta')

subplot(Ncontrol,1,4)
plot(t,u(4,:),'-m')
hold on
if (isempty(U_max) == 0) 
    if(U_max(4) ~= Inf)
        plot(t,U_max(4)*ones(1,Nsim),'-r')
    end
end
if (isempty(U_max) == 0) 
    if(U_min(4) ~= -Inf)
        plot(t,U_min(4)*ones(1,Nsim),'-r')
    end
end
legend('Torque Psi')

%%%%%Plot Variação das Ações de Controle - Delta U
figure
subplot(Ncontrol,1,1)
plot(t,deltau(1,:),'-b')
hold on
if (isempty(deltaU_max) == 0)
    if (deltaU_max(1) ~= Inf)
       plot(t,deltaU_max(1)*ones(1,Nsim),'-r')
    end
end
if (isempty(deltaU_min) == 0)
    if(deltaU_min(1) ~= -Inf)
        plot(t,deltaU_min(1)*ones(1,Nsim),'-r')
    end
end
title('Delta U')
legend('Delta Empuxo')

subplot(Ncontrol,1,2)
plot(t,deltau(2,:),'-g')
hold on
if (isempty(deltaU_max) == 0)
    if(deltaU_max(2) ~= Inf)
        plot(t,deltaU_max(2)*ones(1,Nsim),'-r')
    end
end
if (isempty(deltaU_min) == 0)
    if(deltaU_min(2) ~= -Inf)
        plot(t,deltaU_min(2)*ones(1,Nsim),'-r')
    end
end
legend('Delta Torque Phi')

subplot(Ncontrol,1,3)
plot(t,deltau(3,:),'-k')
hold on
if (isempty(deltaU_max) == 0) 
    if(deltaU_max(3) ~= Inf)
        plot(t,deltaU_max(3)*ones(1,Nsim),'-r')
    end
end
if (isempty(deltaU_min) == 0)
    if(deltaU_min(3) ~= -Inf)
        plot(t,deltaU_min(3)*ones(1,Nsim),'-r')
    end
end
legend('Delta Torque Theta')

subplot(Ncontrol,1,4)
plot(t,deltau(4,:),'-m')
hold on
if (isempty(deltaU_max) == 0)
    if(deltaU_max(4) ~= Inf)
        plot(t,deltaU_max(4)*ones(1,Nsim),'-r')
    end
end
if (isempty(deltaU_min) == 0)
    if(deltaU_min(4) ~= -Inf)
        plot(t,deltaU_min(4)*ones(1,Nsim),'-r')
    end
end
legend('Delta Torque Psi')