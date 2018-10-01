clear all
close all
clc
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
Umax = 0.2;
Umin = -0.4;
tol = 1e-8;
x_inicial = [1 1]';
Xmax = [];%[1.5 1.5];
Xmin = [];%[-1.5 -1.5];
P = dlyap(A,Q);

[H,F] = matrices_cost_function_reformulation(A, B, P, Q, R, Ny, Nu);

[G, W, E, S] = constraints_matrices_reformulation(B, H, F, Nu, Umax, Umin, Xmax, Xmin);

all_possible_const = generate_all_possible_constraints(G, W, S, H, tol);
% %%
% %Calculo de z0 para o ponto inicial
% z0 = optimal_z_mp_QP( G, W, S, H, F, x_inicial, Nu)
% %%
% % z0 = zeros(2,1);
% tol = 1e-8;
% [index, G_tio, W_tio, S_tio] = verify_active_constraints(G, W, S, x_inicial, z0, tol);
%  [A, b, type] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol);
%         [A, b, type] = remove_redundant_constraints(A, b, type, Nu, Nstate);
% indices_regions{1,1} = index;
% %%
% % lambda = -inv(G_tio*inv(H)*G_tio')*(W_tio + S_tio*x_inicial)
% % z = -inv(H)*G'*lambda
% %[A, b, type] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol);
% 
% A = -S;
% b = W;
% type = ones(8,1);
% %%
% [A, b, type] = remove_redundant_constraints(A, b, type, Nu, Nstate);
% 
% [ origem ] = verify_const_origin(A, b, G, W, S, z0);
% 
% index_regions{1} = index;
% for i = 1:size(A,1)
%     if type(i) == 1
%         index_regions{i+1} = [index; origem(i)];
%     else
%         index_regions{i+1} = index(1:(end-1));
%     end
% end
% %%
% figure
% for i = 2:5 
%     [ G_tio, W_tio, S_tio] = build_active_const(G, W, S, index_regions{i})
%     [A, b, type] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol);
%     [A, b, type] = remove_redundant_constraints(A, b, type, Nu, Nstate);
%     [ origem ] = verify_const_origin(A, b, G, W, S, z0);
%     plotregion(-A,-b);
% %     xlim([-1.5 1.5])
% %     ylim([-1.5 1.5])
% end
%%
z0 = zeros(2,1);
Lopt{1,1} = [];
Lcand{1,1} = [];
% z0 = zeros(Nu,1);
Regions = {};
N_zeros = 0;
comb_ruim = {};

while isempty(Lcand) == 0
    disp('Exploradas ')
    length(Lopt)
    disp('Inexploradas ')
    length(Lcand)
    if isempty(Lcand{end,1}) == 0
        [ G_tio, W_tio, S_tio] = build_active_const(G, W, S, Lcand{end,1});
        %[A, b, type] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol);
        [A, b, type, origem] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol);
        if (isempty(A) == 0)
            %[A, b, type] = remove_redundant_constraints(A, b, type, Nu, Nstate);
            
            [A, b, type, origem] = remove_redundant_constraints(A, b, type, origem, Nu, Nstate);
            [Kx, Ku] = define_control(G, W, S, G_tio, W_tio, S_tio, H, F);
            %[ origem ] = verify_const_origin(A, b, G, W, S, z0, all_possible_const);
            %[ origem ] = verify_const_origin(A, b, G, W, S, z0, all_possible_const);
            Lopt = [Lopt; sort(Lcand{end,1})];
        end
    else
        A = -S;
        b = W;
        origem = (1:size(G,1))';
        type = ones(size(G,1),1);
        Kx = (-inv(H)*F');
        Kx = Kx(1,:);
        Ku = 0;
    end
    
    if (isempty(A) == 0)
        
        CR = {A b Kx Ku};
        Regions = [Regions; CR];
        %[ origem ] = verify_const_origin(A, b, G, W, S, z0, all_possible_const);

        L_new_cand = {};
        for i = 1:size(A,1)
            possible_new_set = [];
%             if ((isempty(Lcand{end,1}) ~= 1) && (Lcand{end,1} == origem(i)))
%                 comb_ruim{end+1} = [Lcand{end,1}; origem(i)];
%             end
            
            if (type(i) == 1 ) && (origem(i) ~= 0) && (isempty(Lcand{end,1}) || (Lcand{end,1} ~= origem(i)))
                %index_regions{i+1} = [index; origem(i)];
%                 if isempty(Lcand{end,1}) == 0
%                     if (Lcand{end,1} ~= origem(i)
                
                possible_new_set = [Lcand{end,1}; origem(i)];
            %elseif isempty(Lcand{end-1}) == 0
            elseif type(i) == 2 && origem(i) ~= 0
                %index_regions{i+1} = index(1:(end-1));
                 possible_new_set = Lcand{end,1};
                 possible_new_set = possible_new_set(1:end-1,1);
            else
                if (Lcand{end,1} == origem(i))
                    comb_ruim{end+1} = [Lcand{end,1}; origem(i)];
                end
                N_zeros = N_zeros+1;
            end

            %possible_new_set

            if (check_if_verified(possible_new_set,Lopt) == false) && (isempty(possible_new_set) == 0)
                 L_new_cand{end+1,1} = possible_new_set
                 disp('hue');
                  %L_new_cand = {L_new_cand; possible_new_set}
            end
        %L_new_cand
        end
    else
        L_new_cand = {};
    end
    
    Lcand(end) = [];
    if isempty(L_new_cand) == 0
        if isempty(Lcand) == 1
            Lcand = L_new_cand;
        else
            Lcand = [Lcand; L_new_cand];
        end
    end
%     Lcand{end,1}
end
    

%%
figure(5)
hold on
for i=1:size(Regions,1)
    plotregion(-[Regions{i,1}; 1 0; -1 0; 0 1; 0 -1],-[Regions{i,2}; 1.5 ;1.5 ;1.5 ; 1.5 ])
    xlim([-1.5 1.5])
    ylim([-1.5 1.5])
    i
end

% 
% figure(25)
% x = -1.5:0.01:1.5;
% b = -W;
% A = S;
% % i=1;
% % hue = ( b(i) - A(i,1)*x)/A(i,2);
% % %%
% for i = 1:size(b,1)
%     %a1x+a2y <= b
%     %a2*y <= b - a1*x
%     %y <= (b - a1*x)/a2
%     y(i,:) = ( b(i) - A(i,1)*x)/A(i,2); 
%     plot(x,y(i,:))
%     hold on
% end
% 
% %%
% figure(28)
% x = -1.5:0.01:1.5;
% 
% for i = 1:size(b,1)
%    for j = 1:
%     
%     
% end
% b = -W(1,1)-W(2,1);
% b = [b; -W(3,1)-W(4,1)];
% b = [b; -W(5,1)-W(6,1)];
% b = [b; -W(6,1)-W(8,1)];
% 
% A = S(1,:)+S(2,:);
% A = [A; S(3,:)+S(4,:)];
% A = [A; S(5,:)+S(6,:)];
% A = [A; S(7,:)+S(8,:)];
% % i=1;
% % hue = ( b(i) - A(i,1)*x)/A(i,2);
% % %%
% for i = 1:size(b,1)
%     %a1x+a2y <= b
%     %a2*y <= b - a1*x
%     %y <= (b - a1*x)/a2
%     y(i,:) = ( b(i) - A(i,1)*x)/A(i,2); 
%     plot(x,y(i,:))
%     hold on
% end