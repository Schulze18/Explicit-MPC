clear all
%close all
clc
%%
%System from Example 1 Bemporad 2002
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
%P dado pela solução de Lyapunov Eq 4
P = dlyap(A,Q);

[H, F] = matrices_cost_function_reformulation(A, B, P, Q, R, Ny, Nu)
Xmax = [1.5 1.5];
Xmin = [-1.5 -1.5];
[G, W, E, S] = constraints_matrices_reformulation(B, H, F, Nu, Umax, Umin, Xmax, Xmin)
%%

% G = [ 1  0;
%      -1  0;
%       0  1;
%       0  -1];
% W = [Umax; -Umin; Umax; -Umin];
% 
% E = zeros(Nc*Nu,Nu);
% 
% %Adição da restrição no estado 1.5 < x1,x2 < 1.5
% W = [W; 1.5; 1.5; 1.5; 1.5];
% E = [E; -1 0; 0 -1; 1 0; 0 1];
% G = [G; zeros(4,2)];

%%Criação de CR0 e primeira divisão das areas
%% Calculo z0
%S = E + G*inv(H)*F';
z0 = optimal_z_mp_QP( G, W, S, H, F, x_inicial, Nu)
%z0 = fcnKKT(H, F, G, E, W, x_inicial)
%% Obtenção G_tio, W_tio e S_tio de CR0

tol = 1e-8;
[G_tio, W_tio, S_tio] = verify_active_constraints(G, W, S, x_inicial, z0, tol);

[A_CR0 b_CR0] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol);
[Kx, Kc] = define_control(G, W, S, G_tio, W_tio, S_tio, H, F);
Regions{1,1} = A_CR0;
Regions{1,2} = b_CR0;
Regions{1,3} = Kx;
Regions{1,4} = Kc;


out_X{1,1} = [1 0; 0 1; -1 0; 0 -1];
out_X{1,2} = [Xmax' ; -Xmin'];

i=1;
Nx = 4;          %Quantidade de linhas que descrevem a restrição no espaço
%%
%CR0_rest = new_rest_regions( A_CR0, b_CR0, out_X, out_X);
CR0_rest = find_rest_regions(A_CR0,b_CR0, Nu, Nstate,out_X);
%%
figure(1)
plotregion(-A_CR0,-b_CR0)
hold on
for i = 1:3
   plotregion(-CR0_rest{i,1},-CR0_rest{i,2})
end
xlim([-1.5 1.5])
ylim([-1.5 1.5])

%%
new_Regions = partition_rest_regions(CR0_rest, G, W, S, H, F, tol, Nu, Nstate, 1);
Regions = [Regions; new_Regions];

%%
figure(5)
for i=1:size(new_Regions,1)
    plotregion(-new_Regions{i,1},-new_Regions{i,2}) 
    xlim([-1.5 1.5])
    ylim([-1.5 1.5])
end
plotregion(-A_CR0,-b_CR0)
xlim([-1.5 1.5])
ylim([-1.5 1.5])
