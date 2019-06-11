%Explicit MPC for Iris 3DR Quadcopter for z,theta,phi and psi control
% clear all
% close all
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

%%
Ny = 10;
Nu = 2;

%Weight matrices
Q = diag([3, 0.1, 0.1, 0.05]);
Ru = 0.5*diag([0.001, 0.02, 0.02, 0.05]);
Rdu = 0.5*diag([0.005, 0.05, 0.05, 0.05]);

%Constraints
delta_w = 100;
torque_phi_max = 4*KT*ly*wo*delta_w;
torque_theta_max = 4*KT*lx*wo*delta_w;
torque_psi_max = 4*KD*wo*delta_w;
delta_it = 4;

deltaU_max = [8*m torque_phi_max/delta_it torque_theta_max/delta_it torque_psi_max/delta_it]';
deltaU_min = [-4*m -torque_phi_max/delta_it -torque_theta_max/delta_it -torque_psi_max/delta_it]';

U_max = [2*m*g torque_phi_max torque_theta_max torque_psi_max]';
U_min = [-m*g -torque_phi_max -torque_theta_max -torque_psi_max]';


%% Controller formulation
model = LTISystem('A', A, 'B', B, 'C', C);

model.y.with('reference');
model.y.reference = 'free';
model.u.with('deltaPenalty');

model.u.deltaPenalty = QuadFunction(Rdu);
model.u.penalty = QuadFunction(Ru);
model.y.penalty = QuadFunction(Q);

model.u.with('deltaMin');
model.u.with('deltaMax');
model.u.deltaMax = deltaU_max;
model.u.deltaMin = deltaU_min;
model.u.max = U_max;
model.u.min = U_min;

%Set Control Horizonte
model.u.with('block');
model.u.block.from = Nu;
model.u.block.to = Ny;

ctrl = MPCController(model, Ny);
%%
tic
expmpc = ctrl.toExplicit();
toc
