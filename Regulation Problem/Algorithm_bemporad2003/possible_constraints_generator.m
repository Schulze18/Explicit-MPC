clear all
clc
close all

A = [0.7326 -0.0861;
     0.1722 0.9909];
B = [0.0609; 0.0064];
C = [0 1.4142];
Q = eye(2);
R = 0.01;
Ny = 2;
Nu = 2;
Nc = 2;
Nstate = 2;
Umax = 2;
Umin = -2;
x_inicial = [1 1]';
Xmax = [1.5 1.5];
Xmin = [-1.5 -1.5];
P = dlyap(A,Q);
tol = 1e-5;
[H,F] = matrices_cost_function_reformulation(A, B, P, Q, R, Ny, Nu);

[G, W, E, S] = constraints_matrices_reformulation(B, H, F, Nu, Umax, Umin, Xmax, Xmin);

possible_const = {};
for i = 1:size(G,1)
    G_tio = G(i,:);
    S_tio = S(i,:);
    W_tio = W(i,:);
    [A, b, type] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol);

    possible_const{i,1} = A;
    possible_const{i,2} = b;
end