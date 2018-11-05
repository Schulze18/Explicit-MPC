%Regulation to a non-zero setpoint
%Use of an augmented state space model
clc
clear all
close all
tic

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

%Continuous State Space
Ac = [-Az/m 0;
      1  0];
Bc = [1; 0];
Cc = [0 1];

%Discrete State Space with Ts = 10ms
A = [0.09982 0;
     0.0100 1];
B = [0.01; 0];
C = [0 1];

Nstate = size(A,1);
Ncontrol = size(B,2);

%Augmented State Space
% Aa = [A zeros((Nstate),1); -C 0]; 
% Ba = [B; 0];
% Ca = C;
Aca = [Ac zeros((Nstate),1); -Cc 0]; 
Bca = [B; 0];
[Aa, Ba] = c2d(Aca,Bca,Ts);
Ca = C;

Nstate = size(Aa,1);
Ncontrol = size(Ba,2);


%Explicit MPC Parameters
%Q = [0.17 0 0; 0 0.07 0; 0 0 1/1.5^2];%eye(Nstate);
R = 0.0001;
Q = 10;
P = Q;
Ny = 10;
Nu = 10;
Umax = 2*g;
Umin = -g;
Xmax = [30 11];
Xmin = [-3 -1];
Refmax = 11;
Refmin = -1;
tol = 1e-8;
%P = zeros(3);%dlyap(Aa,Q);

%% Generate Basic Matrices
[H, F, Sx, Su, Syx, Syu] = regulation_matrices_cost_function(A, B, C, P, Q, R, Ny, Nu);

%%
[G, W, E, S, num_Gu] = regulation_constraints_reformulation(A, B, H, F, Sx, Su, Ny, Nu, Umax, Umin, Xmax, Xmin, Refmax, Refmin);

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
            [ G_tio, W_tio, S_tio, index, status_infesiable] = resolve_degenerancy(G, W, S, H, F, Lcand{end,2}, Lcand{end,3}, Nu, Nstate, Lcand{end,1}, tol);
            disp('Degen')
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
              

                [A, b, type, origem] = remove_redundant_constraints(A, b, type, origem, Nu, Nstate);
                Lcand{end,2} = A;
                Lcand{end,3} = b;
                [Kx, Ku] = define_control(G, W, S, G_tio, W_tio, S_tio, H, F);
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
        [A, b, type, origem] = remove_redundant_constraints(A, b, type, origem, Nu, Nstate);
        Kx = (-inv(H)*F');
        Kx = Kx(1,:);
        Ku = 0;
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
    fprintf('Exploradas: %d\nInexploradas: %d\nRegioes: %d\n\n',length(Lopt),length(Lcand),length(Regions));
end
toc


%%
figure
hold on
for i = 1:size(Regions,1)
    P = Polyhedron('A', Regions{i,1}, 'b', Regions{i,2});
    P.plot();
    xlim([Xmin(1) Xmax(1)])
    ylim([Xmin(2) Xmax(2)])
    zlim([Refmin Refmax])
end
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
for i = 1:size(Regions,1)
    regions_union(i) = Polyhedron('A',Regions{i,3},'b',Regions{i,4});
end
regions_union = Union(regions_union);
