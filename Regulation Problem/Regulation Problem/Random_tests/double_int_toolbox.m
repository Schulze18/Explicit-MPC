clear all
close all


Ts = 0.05;
A = [1 Ts; 0 1];
B = [Ts^2; Ts];

H = [1.079 0.076; 0.076 1.073];
F = [1.109 1.036; 1.573 1.517];
G = [1 0; 0 1; -1 0; 0 -1; 0.05 0; 0.05 0.05; -0.05 0; -0.05 -0.05];
W = [1 1 1 1 0.5 0.5 0.5 0.5]';
S = [1 0.9 -1 -0.9 0.1 0.1 -0.1 -0.1; 
     1.4 1.3 -1.4 -1.3 -0.9 -0.9 0.9 0.9]';
E = S - G*inv(H)*F;
 
 
Umin = -1;
Umax = 1;
X2max = 0.5;
X2min = -0.5;

Ny = 6;
Nu = Ny;
R = 1;
Q = diag([1 0]);

model = LTISystem('A', A, 'B', B);
model.x.penalty = QuadFunction(Q);
%model.x.with('setConstraint');
%poly_domain = Polyhedron('A', [0 1; 0 -1;], 'b', [X2max; -X2min]);
%model.x.setConstraint = poly_domain;
model.x.max = [Inf;X2max];
model.x.min = [-Inf;X2min];

model.u.min = Umin;
model.u.max = Umax;
model.u.penalty = QuadFunction(R);

ctrl = MPCController(model, Ny);

explicit_test = ctrl.toExplicit