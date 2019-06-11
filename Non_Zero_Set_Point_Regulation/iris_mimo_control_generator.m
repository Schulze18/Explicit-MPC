%Explicit MPC for Iris 3DR Quadcopter for z,theta,phi and psi control
clear all
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
 
 %Explicit MPC Parameters
 Q = diag([1,1,1,1]);
 R = diag([0.0001,0.001,0.001,0.001]);
 Ny = 5;
 Nu = 5;
%  U_max = [2*m*g 0.5 Inf Inf];
%  U_min = [-m*g -0.5 -Inf -Inf];
 U_max = [2*m*g Inf Inf Inf];
 U_min = [-m*g -Inf -Inf -Inf];
 
%  U_max = [2*g 20 20 20]';
%  U_min = [-g -g -g -g]';
 Ref_max = [];%[30 Inf Inf Inf]';
 Ref_min = [];%[-1 -Inf -Inf -Inf]';
%  Ref_max = 30*ones(1,4)';
%  Ref_min = [-1 -5 -10 -11]';
 X_max = [];%[Inf 1 2 Inf Inf Inf Inf Inf]';
 X_min = [];%-[Inf 10 Inf Inf Inf Inf Inf Inf]';
 Y_max = [];%[10 Inf Inf Inf]';
 Y_min = [];%[-1.5 -Inf -Inf -Inf]';
 
 % Code parameters
 tol = 1e-7;
 n_plot = 20;
 last_plot = 0;
 
 %%
 [H, F, Sx, Su, Qlinha, Rlinha, Clinha] = setpoint_generic_matrices_cost_function(A, B, C, Q, R, Nstate, Ncontrol, Nout, Ny, Nu);
 %%
 [G, W, E, S, num_Gu] = setpoint_generic_constraints(Sx, Su, Clinha, H, F, Nstate, Ncontrol, Nout, Nref, Ny, Nu, U_max, U_min, Ref_max, Ref_min, X_max, X_min, Y_max, Y_min);
 %[G_u, W_u, E_u, G_u_max, W_u_max, E_u_max, G_r, W_r, E_r,num_Gu] = setpoint_generic_constraints(Sx, Su, Clinha, H, F, Nstate, Ncontrol, Nout, Nref, Ny, Nu, U_max, U_min, X_max, X_min, Ref_max,Ref_min);
%% Loop variables Initialization
Lopt{1,1} = [];Lcand{1,2} = []; Lcand{1,3} = [];
%Lcand{1,1} = [6]; Lcand{1,2} = []; Lcand{1,3} = [];
%Lcand{1,1} = [1,2,3,4]'; Lcand{1,2} = []; Lcand{1,3} = [];
%Lopt{1,1} = [1,2,3,4,5];
Regions = {};
N_zeros = 0;
comb_ruim = {};
n=0;
fprintf('Exploradas: %d\nInexploradas: %d\nRegioes: %d\n\n',0,1,0);
tic
while isempty(Lcand) == 0
    %fprintf('Exploradas: %d\nInexploradas: %d\nRegioes: %d\n\n',length(Lopt),length(Lcand),length(Regions));
    n = n+1;
    status_infesiable = 0;
    if isempty(Lcand{end,1}) == 0
        
        %Verify if the index were already tested
        if check_if_verified(Lcand{end,1}, Lopt) == 1
            status_infesiable = 1;
        else
            [ G_tio, W_tio, S_tio] = build_active_const(G, W, S, Lcand{end,1});
        end

        if (rank(G_tio) < size(G_tio,1)) && (rank(G_tio) < size(G_tio,2))% && (size(Lcand{end,1},1) > 1)
            [ G_tio, W_tio, S_tio, index, status_infesiable] = resolve_degenerancy(G, W, S, H, F, Lcand{end,2}, Lcand{end,3}, Nstate+Nref, Ncontrol, Nout, Ny, Nu, Lcand{end,1}, tol);
            %disp('Degen')
            index = sort(index);
            %Add the resulting index to the optimized list
            if ((isequal(index,Lcand{end,1})==0) && isempty(index)==0)
                if (n>1)
                    %Lopt{end+1,1} = sort(index);
                    Lopt{end+1,1} = index;
                else
                    %Lopt{1,1} = sort(index);
                    Lopt{1,1} = index;
                end
            end
            
        end
        
        if status_infesiable == 0
            [A, b, type, origem] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol);
             if ((isempty(A) == 0) && (sum((sum(isnan(A)))) == 0) && (sum((sum(isinf(A)))) == 0)) %Soma de todos os elementos de isnan(A)
              

                [A, b, type, origem] = remove_redundant_constraints(A, b, type, origem, Nu, Nstate +  Nref);
                Lcand{end,2} = A;
                Lcand{end,3} = b;
                [Kx, Ku] = define_control(G, W, S, G_tio, W_tio, S_tio, H, F, Ncontrol);
             end           
        end
        %Lopt = [Lopt; sort(Lcand{end,1})];
         if (n>1)
             Lopt{end+1,1} = sort(Lcand{end,1});
         else
             Lopt{1,1} = sort(Lcand{end,1});
         end
    else
        A = -S;
        b = W;
        origem = (1:size(G,1))';
        type = ones(size(G,1),1);
        %type(5:end) = 2;
        [A, b, type, origem] = remove_redundant_constraints(A, b, type, origem, Nu, Nstate +  Nref);
        Kx = (-inv(H)*F');
        Kx = Kx(1:Ncontrol,:);
        Ku = zeros(Ncontrol,1);
% % %         Lopt = [Lopt; []];
% %         Lopt{end+1,1} = [];
        if (n>1)
             Lopt{end+1,1} = [];
        else
             Lopt{1,1} = [];
        end
    end
    
    if ((isempty(A) == 0) && (status_infesiable == 0) && (sum((sum(isnan(A)))) == 0) && (sum((sum(isinf(A)))) == 0))
        
        CR = {A b Kx Ku};
        Regions = [Regions; CR];
        

        L_new_cand = {};
        for i = 1:size(A,1)
            possible_new_set = [];
            flag_repeticao = 0;
            indices_cand = Lcand{end,1}; 
            
%             for j = 1:length(indices_cand)
%                 if indices_cand(j) == origem(i)
%                     flag_repeticao = 1;
%                 end
%             end
            
            %if (type(i) == 1 ) && (origem(i) ~= 0) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))
            if (type(i) == 1 ) && (origem(i) ~= 0 && origem(i) <= num_Gu) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))   
                possible_new_set = [Lcand{end,1}; origem(i)];
          
            %elseif type(i) == 2 && origem(i) ~= 0
            %elseif (type(i) == 2) && (origem(i) ~= 0) && (origem(i) > num_Gu)
            %elseif (type(i) == 2) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))
            %elseif (type(i) == 2 || origem(i) > num_Gu) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))
            elseif (type(i) == 2 || origem(i) > num_Gu) && (isempty(Lcand{end,1})==0)
                 possible_new_set = Lcand{end,1};
                 %possible_new_set = possible_new_set(1,1:end-1);
                 possible_new_set = possible_new_set(1:end-1,1);
            else
                if (Lcand{end,1} == origem(i))
                    comb_ruim{end+1} = [Lcand{end,1}; origem(i)];
                end
                N_zeros = N_zeros+1;
            end

            %possible_new_set

            %if (check_if_verified(possible_new_set,Lopt) == false) && (isempty(possible_new_set) == 0)
            if ((check_if_verified(possible_new_set,Lopt) == false) && (isempty(L_new_cand) || check_if_verified(possible_new_set,L_new_cand) == false) && (check_if_verified(possible_new_set,Lcand) == false))
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
    
    if (length(Lopt) - last_plot) > n_plot
        fprintf('Exploradas: %d\nInexploradas: %d\nRegioes: %d\n\n',length(Lopt),length(Lcand),length(Regions));
        last_plot = length(Lopt);
    end
end
toc
fprintf('Exploradas: %d\nInexploradas: %d\nRegioes: %d\n\n',length(Lopt),length(Lcand),length(Regions));

%%
% figure
% hold on
% for i = 1:size(Regions,1)
%     P = Polyhedron('A', Regions{i,1}, 'b', Regions{i,2});
%     P.plot();
%     xlim([Xmin(1) Xmax(1)])
%     ylim([Xmin(2) Xmax(2)])
%     zlim([Refmin Refmax])
% end
%A_plot = [eye(3); -eye(3)];
%b = [Xmax'; Refmax; -Xmin'; -Refmin];
% figure
% hold on
% for i=1:size(Regions,1)
%    A_plot = Regions{i,1};
%    A_plot = A_plot(:,1:2);
%    b_plot = Regions{i,2};
%    %plotregion(-[Regions{i,1}; 1 0; -1 0; 0 1; 0 -1],-[Regions{i,2}; 1.5 ;1.5 ;1.5 ; 1.5 ])
%    %plotregion(-[Regions{i,1}; 1 0; -1 0; 0 1; 0 -1],-[Regions{i,2}; 15 ;15 ;15 ; 15 ])
%    %plotregion(-Regions{i,1},-Regions{i,2})
%   % plotregion(-[A_plot; 1 0; -1 0; 0 1; 0 -1], -[b_plot; 15 ;15 ;15 ; 15])
%     xlim([-20 20])
%     ylim([-20 20])
%     i
% end

%%
% for i = 1:size(Regions,1)
%     regions_union(i) = Polyhedron('A',Regions{i,3},'b',Regions{i,4});
% end
% regions_union = PolyUnion(regions_union);

