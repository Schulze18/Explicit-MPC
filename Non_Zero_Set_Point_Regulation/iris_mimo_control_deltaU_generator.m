%Explicit MPC for Iris 3DR Quadcopter for z,theta,phi and psi control
clear all
% close all
% clc

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
%Explicit MPC Parameters
% % % Q = diag([5,1,1,1]);
% % % %  Ru = diag([0.00001,0.00001,0.00001,0.00001]);
% % % %  Rdu = diag([0.00005,0.00005,0.00005,0.00005]);
% % % %Q = diag([0.7/10^2, 0.1/0.5^2, 0.1/0.5^2 0.1/0.5^2]);
% % % Ru = 0.01*diag([0.7/(2*m*g)^2, 0.1, 0.1, 0.1]);
% % % Rdu = 0.01*diag([0.7/(8*m)^2, 0.1/0.1^2, 0.1/0.1^2, 0.1/0.1^2]);
% Q = diag([3,0.05,0.05,0.03]);
% Ru = 0.2*diag([0.0005,0.02,0.02,0.04]);
% Rdu = 0.5*diag([0.005,0.05,0.05,0.05]);
% % % % % % % % % % % % Q = diag([10, 0.01, 0.01, 0.01]);
% % % % % % % % % % % % Ru = 0.2*diag([1,100,100,100]);
% % % % % % % % % % % % Rdu = 0.5*diag([1,100,100,100]);

%Weigh Matrices
% Q = diag([3,0.11,0.098,0.05]);
% Ru = 0.5*diag([0.001,0.021,0.019,0.048]);
% Rdu = 0.5*diag([0.0052,0.048,0.053,0.055]);
Q = diag([3,0.10,0.10,0.05]);
Ru = 0.5*diag([0.001,0.02,0.02,0.05]);
Rdu = 0.5*diag([0.005,0.05,0.05,0.05]);

%Horizons
Ny = 10;
Nu = 2;
 
% angle_speed_max = pi/4;
% angle_acel_max = angle_speed_max/5;

%Constraints calculation
delta_w = 50;
tau_phi_max = 4*KT*lx*wo*delta_w;
tau_theta_max = 4*KT*ly*wo*delta_w;
tau_psi_max = 4*KD*wo*delta_w;
delta_it = 4;

deltaU_max = [];%[8*m Inf Inf Inf]';
deltaU_min = [];%[-4*m -Inf -Inf -Inf]';
% deltaU_max = [8*m tau_phi_max/delta_it tau_theta_max/delta_it tau_psi_max/delta_it]';
% deltaU_min = [-4*m -tau_phi_max/delta_it -tau_theta_max/delta_it -tau_psi_max/delta_it]';
% deltaU_max = [8*m tau_phi_max/delta_it tau_theta_max/delta_it Inf]';
% deltaU_min = [-4*m -tau_phi_max/delta_it -tau_theta_max/delta_it -Inf]';
% deltaU_max = [8*m 1.2*angle_acel_max*Ixx 0.7*angle_acel_max*Iyy 1.5*angle_acel_max*Izz]';
% deltaU_min = [-4*m -0.9*angle_acel_max*Ixx -1.3*angle_acel_max*Iyy -2*angle_acel_max*Izz]';
% deltaU_max = [];%[8*m 0.1 0.1 0.1]';
% deltaU_min = [];%[-4*m -0.1 -0.1 -0.1]';


%deltaU_max = [8*m 1.2*angle_acel_max*Ixx Inf Inf]';
%deltaU_min = [-4*m -0.9*angle_acel_max*Ixx -Inf -Inf]';

% U_max = [2*m*g tau_phi_max Inf Inf]';
% U_min = [-m*g -tau_phi_max -Inf -Inf]';
U_max = [2*m*g tau_phi_max tau_theta_max Inf]';
U_min = [-m*g -tau_phi_max -tau_theta_max -Inf]';
% U_max = [2*m*g tau_phi_max tau_theta_max tau_psi_max]';
% U_min = [-m*g -tau_phi_max -tau_theta_max -tau_psi_max]';
% U_max = [2*m*g 1.2*angle_speed_max*Ixx 0.7*angle_speed_max*Iyy 1.5*angle_speed_max*Izz]';
% U_min = [-m*g -0.9*angle_speed_max*Ixx -1.3*angle_speed_max*Iyy -2*angle_speed_max*Izz]';
% U_max = [2*m*g 0.2 0.15 0.1]';
% U_min = [-m*g -0.2 -0.15 -0.1]';


Ref_max = [];%[30 Inf Inf Inf]';
Ref_min = [];%[-1 -Inf -Inf -Inf]';
 
X_max = [];%[Inf 1 2 Inf Inf Inf Inf Inf]';
X_min = [];%-[Inf 10 Inf Inf Inf Inf Inf Inf]';

Y_max = [];%[10 Inf Inf Inf]';
Y_min = [];%[-1.5 -Inf -Inf -Inf]';
 
% Code parameters
tol = 1e-6;
n_plot = 100;
last_plot = 0;

%Solver Options
sdp_opt = sdpsettings;
sdp_opt.solver = 'sdpt3';
sdp_opt.verbose = 0;
sdp_opt.cachesolvers = 1;
sdp_opt.sdpt3.maxit = 30;
sdp_opt.sdpt3.steptol = 1.0000e-5;
sdp_opt.sdpt3.gaptol = 5.000e-5;
 
%%
[H, F, Sx, Su, Sdu, T_du, Qlinha, Rulinha, Rdulinha, Clinha] = setpoint_deltaU_matrices_cost_function(A, B, C, Q, Ru, Rdu, Nstate, Ncontrol, Nout, Ny, Nu);
%%
[G, W, E, S, num_Gu] = setpoint_deltaU_generic_constraints(Sx, Su, Sdu, Clinha, H, F, Nstate, Ncontrol, Nout, Nref, Ny, Nu, deltaU_max, deltaU_min, U_max, U_min, Ref_max, Ref_min, X_max, X_min, Y_max, Y_min);

 %% Loop variables Initialization
Lopt{1,1} = [];Lcand{1,2} = []; Lcand{1,3} = [];Lcand{1,4} = [];
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
%     if n == 22
%         disp('hue')
%     end
    status_infesiable = 0;
    flag_verified = 0;
    flag_x_nan = 0;
    %%%%%% Alteracao
    index = Lcand{end,1};
    if isempty(Lcand{end,1}) == 0
        
        %Verify if the index was already tested
        if check_if_verified(Lcand{end,1}, Lopt) == 1
            flag_verified = 1;
            %status_infesiable = 1;
        else
            flag_verified = 0;
            %status_infesiable = 0;
            [G_tio, W_tio, S_tio] = build_active_const(G, W, S, Lcand{end,1});
        end

        if (rank(G_tio) < size(G_tio,1)) && (rank(G_tio) < size(G_tio,2)  &&  flag_verified == 0)%status_infesiable == 0)% && (size(Lcand{end,1},1) > 1)
            [G_tio, W_tio, S_tio, index, status_infesiable, flag_x_nan] = resolve_degenerancy(G, W, S, H, F, Lcand{end,2}, Lcand{end,3}, Nstate + Nref + Ncontrol, Ncontrol, Nout, Ny, Nu, Lcand{end,1}, tol, Lcand{end,4}, sdp_opt);
            %disp('Degen');
            index = sort(index);
            if isequal(index,Lcand{end,1})
                disp('ruim degen')
            end
            
            
            %Verify if the resulting index was already tested
            if ((check_if_verified(index, Lopt) == 1) && (status_infesiable == 0))
                %status_infesiable = 1;
                Lopt{end+1,1} = sort(Lcand{end,1});
                flag_verified = 1;
            end
            
            %Add the resulting index to the optimized list
            if ((isequal(index,Lcand{end,1})==0) && (isempty(index)==0) && (status_infesiable == 0) &&  (flag_verified == 0))
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
            if ((sum((sum(isnan(A)))) == 1) || (sum((sum(isinf(A)))) == 1))
               disp('ruim') 
            end
            if ((isempty(A) == 0) && (sum((sum(isnan(A)))) == 0) && (sum((sum(isinf(A)))) == 0)) %Soma de todos os elementos de isnan(A)
              
                A_old_test = A;
                b_old_test = b;
               [A, b, type, origem] =  remove_redundant_constraints(A, b, type, origem, Nu, Nstate +  Nref + Ncontrol, sdp_opt);
               if ((size(A_old_test,1) - size(A,1)) > 0)
                    A_old_test;
                    A;
               end
%                 Lcand{end,2} = A;
%                 Lcand{end,3} = b;
                [Kx, Ku] = define_control(G, W, S, G_tio, W_tio, S_tio, H, F, Ncontrol);
            end           
        end
        %Lopt = [Lopt; sort(Lcand{end,1})];
        if (n>1 && flag_verified == 0)
            Lopt{end+1,1} = sort(Lcand{end,1});
        %else
        elseif flag_verified == 0
            Lopt{1,1} = sort(Lcand{end,1});
        end
    else
        A = -S;
        b = W;
        origem = (1:size(G,1))';
        type = ones(size(G,1),1);
        %type(5:end) = 2;
        [A, b, type, origem] = remove_redundant_constraints(A, b, type, origem, Nu, Nstate +  Nref + Ncontrol, sdp_opt);
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
    
    L_new_cand = {};
    if ((isempty(A) == 0) && (status_infesiable == 0) && (sum((sum(isnan(A)))) == 0) && (sum((sum(isinf(A)))) == 0) && (flag_verified == 0))
        
        CR = {A b Kx Ku n Lcand{end,2} Lcand{end,3} flag_x_nan status_infesiable};
        Regions = [Regions; CR];
        
        L_new_cand = {};
        for i = 1:size(A,1)
            possible_new_set = [];
            flag_repeticao = 0;
            %%%%%%%%indices_cand = Lcand{end,1}; 
            indices_cand = index;
            
            %%%%%%%%%%%%%%%%%%
            for j = 1:length(indices_cand)
                if (indices_cand(j) == origem(i) && type(i) == 1)
                    flag_repeticao = 1;
                end
            end
            %%%%%%%%%%%%%%%%%%
            new_index = [];
            %if (type(i) == 1 ) && (origem(i) ~= 0) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))
            if (type(i) == 1 ) && (origem(i) ~= 0 && origem(i) <= num_Gu) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))   
                %%%%%%%%%%%%%%%possible_new_set = [Lcand{end,1}; origem(i)];
                possible_new_set = [index; origem(i)];
                %new_index = origem(i); %Old
                new_index = [Lcand{end,4}; origem(i)];%Only for test - saving the latest index add
                
            %elseif type(i) == 2 && origem(i) ~= 0
            %elseif (type(i) == 2) && (origem(i) ~= 0) && (origem(i) > num_Gu)
            %elseif (type(i) == 2) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))
            %elseif (type(i) == 2 || origem(i) > num_Gu) && (isempty(Lcand{end,1}) || (flag_repeticao == 0))
            
            %elseif (type(i) == 2 || origem(i) > num_Gu) && (isempty(Lcand{end,1})==0)
            elseif (type(i) == 2) && (isempty(Lcand{end,1})==0)
                 %%%%%%%%%%%%%%%%%%%%%%%possible_new_set = Lcand{end,1};
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
            possible_new_set = sort(possible_new_set);

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
    
    if (length(Lopt) - last_plot) > n_plot
        fprintf('Exploradas: %d\nInexploradas: %d\nRegioes: %d\n\n',length(Lopt),length(Lcand),length(Regions));
        last_plot = length(Lopt);
    end
end
toc
fprintf('Exploradas: %d\nInexploradas: %d\nRegioes: %d\n\n',length(Lopt),length(Lcand),length(Regions));
figure