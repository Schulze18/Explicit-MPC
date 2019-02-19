%Regulation to a non-zero setpoint Control Implementation
%Use of an augmented state space model

%close all
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

%Continuous State Space
Ac = [-Az/m 0;
      1  0];
Bc = [1; 0];
Cc = [0 1];


%Continuous State Space
Ac = [-Az/m 0;
      1  0];
Bc = [1; 0];
Cc = [0 1];

%Discrete State Space with Ts = 10ms
% A = [0.09982 0;
%      0.0100 1];
% B = [0.01; 0];
% C = [0 1];

A =  [0.7326 -0.0861;
     0.1722 0.9909];
B = [0.0609;
    0.0064];
C = [0 1.4142];
Ts = 0.1;
Nstate = size(A,1);
Ncontrol = size(B,2);

Nsim = 100;
x = zeros(Nstate,Nsim);
y = zeros(Nsim,1);
u = y;
vet_index = y;
erro = zeros(1,Nsim);
t = (0:(Nsim-1))*Ts;
x(:,1) = [0.5 2.5]';
ref = 1;
cont_out = 0;
cont_reg = 0;
vetor_repeticao = erro;
%%
for i = 1:Nsim
    index = 0;
    y(i) = C*x(:,i); 
    erro(i) = ref - y(i); 
    cont_reg(i) = 0;
    for j = 1:size(Regions,1)
        A_CRi = Regions{j,1};
        b_CRi = Regions{j,2};
        flag = 0;       
        for k = 1:size(A_CRi,1)
            if(A_CRi(k,:)*x(:,i) > b_CRi(k))
            %if((A_CRi(k,:)*[x(:,i); ref]) > b_CRi(k))
%                 if flag == 0 
%                     cont_out = cont_out + 1;
%                 end
                flag = 1;
            end
        end
        if flag == 0
            cont_reg(i) = cont_reg(i) + 1;
            index = j;
        end
    end
    vet_index(i) = index;
    %u_calc = Regions{index,3}*[x(:,i); ref] + Regions{index,4};

    u_calc = Regions{index,3}*x(:,i) + Regions{index,4};
    % 
     %u_calc = test_toolbox_5_5([x(:,i); ref]);
%    u_calc = expmpc.evaluate([x(:,i); ref]);

    u(i) = u_calc(1,1);
    
%     u_calc = test_toolbox_10_10([x(:,i); ref ]);
%     u(i) = u_calc(1);
    x(:,i+1) = A*x(:,i) + B*u(i);% + 0.1*[rand(1)-rand(1) ; rand(1)-rand(1)];

end
%%
% % figure
% % subplot(3,1,1)
% % plot(t,y)
% % hold on
% % plot(t,ref*ones(1,Nsim),'-r')
% % legend('Altura','Referencia')
% % % figure
% % subplot(3,1,2)
% % plot(t,u)
% % legend('control')
% % subplot(3,1,3)
% % plot(t,vet_index)
% % legend('indices')
% % % figure
% % % plot(t,cont_reg)
% % figure
% % subplot(2,1,1)
% % plot(t,x(1,1:(end-1)))
% % legend('zp')
% % subplot(2,1,2)
% % plot(t,x(2,1:(end-1)))
% % legend('z')
%%
figure
subplot(3,1,1)
plot(t,x(1,1:Nsim))
legend('x1')
subplot(3,1,2)
plot(t,x(2,1:Nsim))
legend('x2')
subplot(3,1,3)
plot(t,u(1:Nsim))
legend('Input')

%%
figure(1)
plot(x(1,1),x(2,1),'*k')
plot(x(1,1:Nsim),x(2,1:Nsim),'k')
plot(x(1,Nsim),x(2,Nsim),'*k')