clc

%Discrete State Space with Ts = 10ms
A = [0.09982 0;
     0.0100 1];
B = [0.01; 0];
C = [0 1];

%Explicit MPC Parameters
%Q = [0.17 0 0; 0 0.07 0; 0 0 1/1.5^2];%eye(Nstate);
R = 0.0001;
Q = 10;
Ny = 10;
Nu = 10;
Umax = 2*g;
Umin = -g;
Xmax = [30 11];
Xmin = [-3 -1];
Refmax = 11;
Refmin = -1;
tol = 1e-8;


model = LTISystem('A', A, 'B', B, 'C', C);%, 'Ts', 0.01);
model.x.max = Xmax';
model.x.min = Xmin';
% Px = Polyhedron('A',[eye(2);-eye(2)],'b',[Xmax'; -Xmin']);
% model.x.with('setConstraint');
% model.x.setConstraint = Px;

model.y.with('reference');
model.y.reference = 'free';
mode.y.max = Refmax;
mode.y.min = Refmin; 

% Py = Polyhedron('A',[eye(1);-eye(1)],'b',[Refmax; -Refmin]);
% model.y.with('setConstraint');
% model.y.setConstraint = Py;

% Pu = Polyhedron('A',[eye(1);-eye(1)],'b',[Umax; -Umin]);
% model.u.with('setConstraint');
% model.u.setConstraint = Pu;
model.u.max = Umax;
model.u.min = Umin;

model.u.penalty = QuadFunction(R);
model.y.penalty = QuadFunction(Q);
% 
% model.u.with('block'); 
% model.u.block.from = Nu+1;
% model.u.block.to = Ny;

ctrl = MPCController(model, Ny);
%%
expmpc = ctrl.toExplicit();

%%
figure
hold on
for i = 1:size(expmpc.optimizer.Set,1)
   P_toolbox = Polyhedron('A',expmpc.optimizer.Set(i,1).A,'b',expmpc.optimizer.Set(i,1).b);
   P_toolbox.plot();
   xlim([Xmin(1) Xmax(1)])
   ylim([Xmin(2) Xmax(2)])
   zlim([Refmin Refmax])
end

%%
control_toolbox = {}
for i = 1:expmpc.optimizer.Num
        control_toolbox{i,1} = expmpc.optimizer.Set(i).Functions('obj').F;
        control_toolbox{i,2} = expmpc.optimizer.Set(i).Functions('obj').g;
end

%%
control_toolbox = {};
for i = 1:size(Regions,1)
     control_toolbox{i,1} = Regions{i,3};
     control_toolbox{i,2} = Regions{i,4};
end
%%
soma_toolbox={}
for i = 1:size(control_toolbox,1)
    flag = 0;
    for j = 1:size(control_toolbox,1)
        if ((sum(abs(control_toolbox{i,1}-control_toolbox{j,1})) < 1e-3) && (sum(abs(control_toolbox{i,1}-control_toolbox{j,1})) > -1e-3) && j~=i && (sum(abs(control_toolbox{i,2}-control_toolbox{j,2})) < 1e-3) && (sum(abs(control_toolbox{i,2}-control_toolbox{j,2})) > -1e-3)) 
            if (flag==0)
                soma_toolbox{i,2} = j;
            else
                soma_toolbox{i,2} = [soma_toolbox{i,2};j];
            end
            flag = 1;
        end
    end
    soma_toolbox{i,1} = flag;
 end

