%Regulation problema example 1 bemporad with toolbox
A = [0.7326 -0.0861;
     0.1722 0.9909];
B = [0.0609; 0.0064];
C = [0 1.4142];
Q = eye(2);
R = 0.01;
Umax = 2;
Umin = -2;
Xmax = [1.5 1.5]';
Xmin = [-1.5 -1.5]';
P = dlyap(A,Q);
x_inicial = [1 1]';


model = LTISystem('A', A, 'B', B, 'C', C);
model.x.min = Xmin;
model.x.max = Xmax;
model.u.min = Umin;
model.u.max = Umax;

model.x.penalty = QuadFunction(Q);
model.u.penalty = QuadFunction(R);
model.x.with('terminalPenalty');
model.x.terminalPenalty = QuadFunction(P);

ctrl = MPCController(model,Ny);
explicit = ctrl.toExplicit();
