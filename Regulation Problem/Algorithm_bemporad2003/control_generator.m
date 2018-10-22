clear all
%close all
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
Umax = 2;
Umin = -2;
tol = 1e-8;
%reta: x2 = -0.9213*x1 + 0.1278
% %GU < W + Ex
% Ex >-W
% E = [0.9213 1]; W= 0.1278
x_inicial = [1 1]';
Xmax = 2*[1.5 1.5];
Xmin = 2*[-1.5 -1.5];
P = dlyap(A,Q);
%P = Q;
%%
[H, F, Sx, Su] = matrices_cost_function_reformulation(A, B, P, Q, R, Ny, Nu);
%Su(1:2,:) = [];Sx(1:2,:) = [];
[G, W, E, S] = constraints_matrices_reformulation(A, B, H, F, Sx, Su, Ny, Nu, Umax, Umin, Xmax, Xmin);
%W = W(1:12,:);
%%
%all_possible_const = generate_all_possible_constraints(G, W, S, H, tol);
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
Lcand{1,1} = []; Lcand{1,2} = []; Lcand{1,3} = [];
% z0 = zeros(Nu,1);
Regions = {};
N_zeros = 0;
comb_ruim = {};

while isempty(Lcand) == 0
    fprintf('Exploradas: %d\nInexploradas: %d\nRegioes: %d\n\n',length(Lopt),length(Lcand),length(Regions));
    %disp(['Exploradas: ', length(Lopt),' Inexploradas: ',length(Lcand),' Regioes: ', length(Regions)])
%     disp('Exploradas ')
%     length(Lopt)
%     disp('Inexploradas ')
%     length(Lcand)
     status_infesiable = 0;
    if isempty(Lcand{end,1}) == 0
        [ G_tio, W_tio, S_tio] = build_active_const(G, W, S, Lcand{end,1});
        %[A, b, type] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol);
        
        %status_infesiable = 0;
        if rank(G_tio) < size(G_tio,1) && rank(G_tio) < size(G_tio,2)
            %[ G_tio, W_tio, S_tio, index, status_infesiable] = resolve_degenerancy(G, W, S, H, F, A, b, Nu, Nstate, Lcand{end,:}, tol);
            [ G_tio, W_tio, S_tio, index, status_infesiable] = resolve_degenerancy(G, W, S, H, F, Lcand{end,2}, Lcand{end,3}, Nu, Nstate, Lcand{end,1}, tol);
            disp('UEHUEHEHUEHUEHEHUEHUE')
        end
        
        if status_infesiable == 0
            [A, b, type, origem] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol);
             if ((isempty(A) == 0) && (sum((sum(isnan(A)))) == 0) && (sum((sum(isinf(A)))) == 0)) %Soma de todos os elementos de isnan(A)
                %[A, b, type] = remove_redundant_constraints(A, b, type, Nu, Nstate);

                [A, b, type, origem] = remove_redundant_constraints(A, b, type, origem, Nu, Nstate);
                [Kx, Ku] = define_control(G, W, S, G_tio, W_tio, S_tio, H, F);
                %[ origem ] = verify_const_origin(A, b, G, W, S, z0, all_possible_const);
                %[ origem ] = verify_const_origin(A, b, G, W, S, z0, all_possible_const);
             else 
                 %Lopt = [Lopt; sort(Lcand{end,1})];
             end
            
        end
            Lopt = [Lopt; sort(Lcand{end,1})];
%         end
    else
        A = -S;
        b = W;
        origem = (1:size(G,1))';
        type = ones(size(G,1),1);
        %type(5:end) = 2;
        [A, b, type, origem] = remove_redundant_constraints(A, b, type, origem, Nu, Nstate);
        Kx = (-inv(H)*F');
        Kx = Kx(1,:);
        Ku = 0;
% %         A = -S(1:4,:);
% %         b = W(1:4,:);
% %         %origem = (1:size(G,1))';
% %         origem = [1;2;3;4];
% %         %type = ones(size(G,1),1);
% %         type = ones(4,1);
% %         %type = [type; 2*ones(4,1)];
% %         %[A, b, type, origem] = remove_redundant_constraints(A, b, type, origem, Nu, Nstate);
% %         Kx = (-inv(H)*F');
% %         Kx = Kx(1,:);
% %         Ku = 0;
    end
    
    %if (isempty(A) == 0 || status_infesiable == 0)
    if ((isempty(A) == 0) && (status_infesiable == 0) && (sum((sum(isnan(A)))) == 0) && (sum((sum(isinf(A)))) == 0))
        
        CR = {A b Kx Ku};
        Regions = [Regions; CR];
        %[ origem ] = verify_const_origin(A, b, G, W, S, z0, all_possible_const);

        L_new_cand = {};
        for i = 1:size(A,1)
            possible_new_set = [];
%             if ((isempty(Lcand{end,1}) ~= 1) && (Lcand{end,1} == origem(i)))
%                 comb_ruim{end+1} = [Lcand{end,1}; origem(i)];
%             end
            flag_repeticao = 0;
            indices_cand = Lcand{end,1}; 
            
            for j = 1:length(indices_cand)
                if indices_cand(j) == origem(i)
                    flag_repeticao = 1;
                end
            end
            
            %if (type(i) == 1 ) && (origem(i) ~= 0) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))
            if (type(i) == 1 ) && (origem(i) ~= 0 && origem(i) < 5) && (isempty(Lcand{end,1}) || (flag_repeticao == 0)) 
            %index_regions{i+1} = [index; origem(i)];
%                 if isempty(Lcand{end,1}) == 0
%                     if (Lcand{end,1} ~= origem(i)
                
                possible_new_set = [Lcand{end,1}; origem(i)];
            %elseif isempty(Lcand{end-1}) == 0
            %elseif type(i) == 2 && origem(i) ~= 0
            elseif (type(i) == 2) && (origem(i) ~= 0) && (origem(i) >4)
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
                 L_new_cand{end+1,1} = possible_new_set;
                 L_new_cand{end,2} = A;
                 L_new_cand{end,3} = b;
%                  disp('hue');
                  %L_new_cand = {L_new_cand; possible_new_set}
            end
        %L_new_cand
        end
    else
        L_new_cand = {};
    end
    
    Lcand(end,:) = [];
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
figure
hold on
for i=1:size(Regions,1)
   %plotregion(-[Regions{i,1}; 1 0; -1 0; 0 1; 0 -1],-[Regions{i,2}; 1.5 ;1.5 ;1.5 ; 1.5 ])
   %plotregion(-[Regions{i,1}; 1 0; -1 0; 0 1; 0 -1],-[Regions{i,2}; 15 ;15 ;15 ; 15 ])
   plotregion(-Regions{i,1},-Regions{i,2})
   
   
    xlim([-5 5])
    ylim([-5 5])
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