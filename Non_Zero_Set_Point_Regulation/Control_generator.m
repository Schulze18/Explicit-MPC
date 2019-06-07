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
% A = [0.7326 0;
%      0.0100 1];
% B = [0.01; 0];
% C = [0 1];


%Augmented State Space
% Aa = [A zeros((Nstate),1); -C 0]; 
% Ba = [B; 0];
% Ca = C;
% Aca = [Ac zeros((Nstate),1); -Cc 0]; 
% Bca = [B; 0];
% [Aa, Ba] = c2d(Aca,Bca,Ts);
% Ca = C;

A =  [0.7326 -0.0861;
     0.1722 0.9909];
B = [0.0609;
    0.0064];
C = [0 1.4142];

Nstate = size(A,1);
Ncontrol = size(B,2);
Nout = size(C,1);
Nref = 0;

%Explicit MPC Parameters
%Q = [0.17 0 0; 0 0.07 0; 0 0 1/1.5^2];%eye(Nstate);
% R = 0.0001;
% Q = 10;
% P = Q;
% Ny = 5;
% Nu = 2;
% Umax = 2*g;
% Umin = -g;
% Xmax = [30 11];
% Xmin = [-3 -1];
% Refmax = [];%11;
% Refmin = [];%-1;
Q = 20*eye(Nstate);
P = Q;%dlyap(A,Q);
R = 0.01;
Ny = 2;
Nu = 2;
Umax = 1.5;
Umin = -1.5;
Xmax = [];%[2 2];%[30 11];
Xmin = [];%[-2 -2];%[-3 -1];
Refmax = [];
Refmin = [];

%P = zeros(3);%dlyap(Aa,Q);

% Code parameters
tol = 1e-7;
n_plot = 4;
last_plot = 0;

%Solver Options
sdp_opt = sdpsettings;
sdp_opt.solver = 'sdpt3';
sdp_opt.verbose = 0;
sdp_opt.cachesolvers = 1;
sdp_opt.sdpt3.maxit = 30;
sdp_opt.sdpt3.steptol = 1.0000e-5;
sdp_opt.sdpt3.gaptol = 5.000e-5;

%% Generate Basic Matrices
[H, F, Sx, Su, Syx, Syu] = regulation_matrices_cost_function(A, B, C, P, Q, R, Ny, Nu);

%%
[G, W, E, S, num_Gu] = regulation_constraints_reformulation(A, B, H, F, Sx, Su, Ny, Nu, Umax, Umin, Xmax, Xmin, Refmax, Refmin);

%% Loop variables Initialization
Lopt{1,1} = [];Lcand{1,2} = []; Lcand{1,3} = [];Lcand{1,4} = [];
% Lopt{1,1} = [];Lcand{1,2} = []; Lcand{1,3} = [];
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
    flag_verified = 0;
    flag_x_nan = 0;
    index = Lcand{end,1};
    if isempty(Lcand{end,1}) == 0
        
        %Verify if the index were already tested
        if check_if_verified(Lcand{end,1}, Lopt) == 1
            flag_verified = 1;
            %status_infesiable = 1;
        else
             flag_verified = 0;
            %status_infesiable = 0;
            [ G_tio, W_tio, S_tio] = build_active_const(G, W, S, Lcand{end,1});
        end
     
        if (rank(G_tio) < size(G_tio,1)) && (rank(G_tio) < size(G_tio,2))% && (size(Lcand{end,1},1) > 1)
            %[ G_tio, W_tio, S_tio, index, status_infesiable] = resolve_degenerancy(G, W, S, H, F, Lcand{end,2}, Lcand{end,3}, Nu, Nstate, Lcand{end,1}, tol);
            [ G_tio, W_tio, S_tio, index, status_infesiable, flag_x_nan] = resolve_degenerancy(G, W, S, H, F, Lcand{end,2}, Lcand{end,3}, Nstate, Ncontrol, Nout, Ny, Nu, Lcand{end,1}, tol, Lcand{end,4});
            
            
            disp('Degen')
            index = sort(index);
            
             %Verify if the resulting index was already tested
            if ((check_if_verified(index, Lopt) == 1) && (status_infesiable == 0))
                %status_infesiable = 1;
                Lopt{end+1,1} = sort(Lcand{end,1});
                flag_verified = 1;
            end
            
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
        
        if  (status_infesiable == 0) && (flag_verified == 0)
            [A, b, type, origem] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol);
             if ((isempty(A) == 0) && (sum((sum(isnan(A)))) == 0) && (sum((sum(isinf(A)))) == 0)) %Soma de todos os elementos de isnan(A)
              

                [A, b, type, origem] = remove_redundant_constraints(A, b, type, origem, Nu, Nstate, sdp_opt);
%                 Lcand{end,2} = A;
%                 Lcand{end,3} = b;
                [Kx, Ku] = define_control(G, W, S, G_tio, W_tio, S_tio, H, F, Ncontrol);
             end           
        end
        %Lopt = [Lopt; sort(Lcand{end,1})];
        if (n>1 && flag_verified == 0)
             Lopt{end+1,1} = sort(Lcand{end,1});
        elseif flag_verified == 0
             Lopt{1,1} = sort(Lcand{end,1});
        end
    else
        A = -S;
        b = W;
        origem = (1:size(G,1))';
        type = ones(size(G,1),1);
        %type(5:end) = 2;
        [A, b, type, origem] = remove_redundant_constraints(A, b, type, origem, Nu, Nstate, sdp_opt);
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
    
    if ((isempty(A) == 0) && (status_infesiable == 0) && (sum((sum(isnan(A)))) == 0) && (sum((sum(isinf(A)))) == 0) && (flag_verified == 0))    
        CR = {A b Kx Ku n Lcand{end,2} Lcand{end,3} flag_x_nan status_infesiable};
        Regions = [Regions; CR];
        

        L_new_cand = {};
        for i = 1:size(A,1)
            possible_new_set = [];
            flag_repeticao = 0;
            %%%%%%%%indices_cand = Lcand{end,1}; 
            indices_cand = index;
            
            for j = 1:length(indices_cand)
                if indices_cand(j) == origem(i)
                    flag_repeticao = 1;
                end
            end
            
            new_index = [];
            %if (type(i) == 1 ) && (origem(i) ~= 0) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))
            %%if (type(i) == 1 ) && (origem(i) ~= 0 && origem(i) <= num_Gu) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))
            if (type(i) == 1 ) && (origem(i) ~= 0) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))   
                %%%%%%%%%%%%%%%possible_new_set = [Lcand{end,1}; origem(i)];
                possible_new_set = [index; origem(i)];
                %new_index = origem(i); %Old
                new_index = [Lcand{end,4}; origem(i)];%Only for test - saving the latest index add
                
          
            %elseif type(i) == 2 && origem(i) ~= 0
            %elseif (type(i) == 2) && (origem(i) ~= 0) && (origem(i) > num_Gu)
            %elseif (type(i) == 2) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))
            %elseif (type(i) == 2 || origem(i) > num_Gu) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))
            %%elseif (type(i) == 2 || origem(i) > num_Gu) && (isempty(Lcand{end,1})==0)
            elseif (type(i) == 2) && (isempty(Lcand{end,1})==0)
% %                  possible_new_set = Lcand{end,1};
% %                  %possible_new_set = possible_new_set(1,1:end-1);
% %                  possible_new_set = possible_new_set(1:end-1,1);
                 possible_new_set = index;
                 
                 %possible_new_set = possible_new_set(1:end-1,1);
                 possible_new_set(origem(i)) = [];
                  
                 %Only for test - saving the latest index add
                 new_index = Lcand{end,4};
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
                 L_new_cand{end,4} = new_index;
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
    %fprintf('Exploradas: %d\nInexploradas: %d\nRegioes: %d\n\n',length(Lopt),length(Lcand),length(Regions));
    
    if (length(Lopt) - last_plot) > n_plot
        fprintf('Exploradas: %d\nInexploradas: %d\nRegioes: %d\n\n',length(Lopt),length(Lcand),length(Regions));
        last_plot = length(Lopt);
    end
end
toc
fprintf('Exploradas: %d\nInexploradas: %d\nRegioes: %d\n\n',length(Lopt),length(Lcand),length(Regions));

%%
% for i = 1:size(Regions,1)
%     P = Polyhedron('A', Regions{i,1}, 'b', Regions{i,2});
%     P.plot();
%     %xlim([Xmin(1) Xmax(1)])
%     %ylim([Xmin(2) Xmax(2)])
%     %zlim([Refmin Refmax])
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
% regions_union = Union(regions_union);
figure
for i = 1:size(Regions,1)
    %plotregion(-Regions{i,1},-Regions{i,2});
    plotregion(-[Regions{i,1}; 1 0; -1 0; 0 1; 0 -1],-[Regions{i,2}; 10 ;10 ;10 ;10])
    ylim([-3 3])
    xlim([-1 1])
    hold on    
end
ylim([-3 3])
xlim([-1 1])

