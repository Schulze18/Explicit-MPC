%System from Example 1 Bemporad 2002
A = [0.7326 -0.0861;
     0.1722 0.9909];
B = [0.0609; 0.0064];
C = [0 1.4142];

Nsim = 50;
x = zeros(2,50);
y = zeros(50,1);
x(:,1) = [-1.4 1.1]'; 
%%
for i = 1:Nsim
   
    for j = 1:size(Regions,1)
        A_CRi = Regions{j,1};
        b_CRi = Regions{j,2};
        flag = 0;
        for k = 1:size(A_CRi,1)
            if(A_CRi(k,:)*x(:,i) > b_CRi(k))
                flag = 1;
            end
        end
        if flag == 0
            index = j;
        end
    end
    u(i) = Regions{index,3}*x(:,i) + Regions{index,4};
%     A*x(:,i)
%     B*u
    x(:,i+1) = A*x(:,i)+B*u(i);%+0.02*[rand(1) ; rand(1) ];
    y(i) = C*x(:,i); 

end

% close(10)
% close(11)
figure(15)
stairs(x(1,:))
hold on
stairs(x(2,:),'r')
%legend('x1','x2')

figure(16)
hold on
stairs(u)
%legend('u')

figure(1)
plot(x(1,:),x(2,:),'r')
hold on
plot(x(1,1),x(2,1),'*')
ylim([-1.5 1.5])
xlim([-1.5 1.5])



%% MPC Online
Nsim = 100;
x_online = zeros(2,50);
y_online = zeros(50,1);
x_online(:,1) = [-1.4 1.1]'; 

[L,p] = chol(H,'lower');
Linv = inv(L);
for i= 1:Nsim
    Aineq = -G;
    f = F'*x_online(:,i);
    bineq = -W-E*x_online(:,i);
    opt = mpcqpsolverOptions;
    iA0 = false(size(bineq));
    Aeq = [];
    beq = zeros(0,1);
    
    [u_calc,status] = mpcqpsolver(Linv,f,Aineq,bineq,Aeq,beq,iA0,opt);
    u_online(i) = u_calc(1);
    x_online(:,i+1) = A*x_online(:,i)+B*u_online(i);%+0.02*[rand(1) ; rand(1) ];
    y_online(i) = C*x_online(:,i); 
    i
end
figure(15)
hold on
stairs(x_online(1,:),'k')
hold on
stairs(x_online(2,:),'g')
legend('x1','x2','x1_{online}','x2_{online}')
title('Offline x MPCqp')

figure(16)
hold on
stairs(u_online,'r')
legend('u','u_{online}')
title('Offline x MPCqp')

figure(1)
hold on
plot(x_online(1,:),x_online(2,:),'-b')
hold on
plot(x_online(1,1),x_online(2,1),'*')
ylim([-1.5 1.5])
xlim([-1.5 1.5])
title('Offline x MPCqp')


%% Toolbox
model = LTISystem('A', A, 'B', B);
model.x.penalty = QuadFunction(Q);
model.x.with('setConstraint');
poly_domain = Polyhedron('A', [0.9213 1], 'b', 0.1278);
model.x.setConstraint = poly_domain;
model.x.max = Xmax;
model.x.min = Xmin;
model.u.min = Umin;
model.u.max = Umax;
model.u.penalty = QuadFunction(R);

ctrl = MPCController(model, Ny);
loop = ClosedLoop(ctrl, model);
data = loop.simulate(x(:,1), Nsim);

figure(1)
hold on
plot(data.X(1,:),data.X(2,:),'-k')