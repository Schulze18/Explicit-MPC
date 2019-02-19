function [G, W, E, S, num_Gu, G_x2, W_x2, E_x2] = regulation_constraints_reformulation(A, B, H, F, Sx, Su, Ny, Nu, Umax, Umin, Xmax, Xmin, Refmax, Refmin)
%[G, W, E, S] = constraints_matrices_reformulation(A, B, H, F, Nu, Umax, Umin, Xmax, Xmin)
%
%Return the constraints matrices G, W and E from the reformulated cost function with U to
%be opmitze.
%Inputps:
%
%Outpus:

%Algoritm based on the paper "The explicit linear quadratic regulator for
%constrained systems" by A. Bemporad, M. Morari, V. Dua, and E. Pistikopoulos. 

    G_u = [];
    G_x = [];
    G_r = [];
    W_u = [];
    W_x = [];
    W_r = [];
    E_u = [];
    E_x = [];
    E_r = [];
    S = [];
    
    num_state = size(A,2);         %Number of elements in the state vector
    num_control = size(B,2);       %Number of control inputs 
    num_ref = 0;%length(Refmax);
    
    %U constraints
    if isempty(Umax) == 0
        %E_u = zeros(2*num_control*Nu,num_state);        
        %W_u(1:num_control,1) = Umax;
        %W_u((1+num_control):(2*num_control),1) = -Umin;
        %W_u(1:(Nu*num_control),1) = Umax;
        %W_u((end+1):(2*Nu*num_control),1) = -Umin;
        %G_u(1:num_control,1) = ones(size(Umax,2),1);
        %G_u((1+num_control):(2*num_control),1) = -ones(size(Umin,2),1);  
        %G_u(1:num_control,1) = ones(size(Umax,2),1);
        %G_u((end+1):(2*num_control),1) = -ones(size(Umin,2),1); 
        %for i = 1:(Nu-1)
         %   G_u = blkdiag(G_u,G_u);
         %   W_u = [W_u; W_u];
        %end
        E_u = zeros(2*num_control*Nu,num_state + num_ref); 
        G_u = eye(Nu*num_control);
        G_u = [G_u;-G_u];
        num_Gu = size(G_u,1);
        W_u = eye(num_control)*Umax;
        for i = 1:(Nu-1)
            W_u = [W_u; eye(num_control)*Umax];
        end
        for i = 1:Nu
            W_u = [W_u; -eye(num_control)*Umin];
        end
    end
    
    %X constraints
    if isempty(Xmax) == 0
%         G_x = zeros(2*num_state,Nu);
%         E_x = -eye(num_state);
%         E_x = [E_x; eye(num_state)];
%         W_x = Xmax';
%         W_x = [W_x; (-Xmin)'];
        
        %%Faz mais sentido
        G_x = [Su; -Su];
        
        E_x = [-Sx -zeros(size(Sx,1),num_ref); Sx zeros(size(Sx,1),num_ref)];
        W_x = eye(num_state)*Xmax';
        for i = 1:(Ny)
            W_x = [W_x; eye(num_state)*Xmax'];
        end
        for i = 1:(Ny+1)
            W_x = [W_x; -eye(num_state)*Xmin'];
        end
%         
%         Ax = [0 1];
%         
%         E_x2 = [Ax*eye(num_state) 0];
%         for i = 1:Ny
%             E_x2 = [E_x2; -Ax*A^i 0];
%         end
%         
%         G_x2 = zeros((Ny+1),Nu);
%         for j = 1:Nu
%             for i = 1:(Ny+1)
%                 if (i-j-1)>0
%                     G_x2(((i-1)+1):((i)),j) =  [Ax*A^(i-j-1)*B];
%                 elseif (i-j-1) == 0
%                     G_x2(((i-1)+1):i,j) = [Ax*B];
%                 end
%             end
%         end
%         
%         W_x2 = Xmax(2)';
%         for i = 1:(Ny)
%             W_x2 = [W_x2; Xmax(2)'];
%         end
%         for i = 1:(Ny+1)
%             W_x2 = [W_x2; -Xmin(2)'];
%         end
%         
%         E_x = [E_x2; -E_x2];
%         G_x = [G_x2; -G_x2];
%         W_x = W_x2;
%         
    end
    
    if isempty(Refmin) == 0
        G_r = zeros(2*num_ref,num_control*Nu);
        E_r = [zeros(1,num_state) -1; zeros(1,num_state) 1];
        W_r = [Refmax; (-Refmin)'];
    end
    
 
%     W = [W_u; W_x];
%     G = [G_u ; G_x];
%     E = [E_u ; E_x];
    
%     G = G(1:10,:);
%     W = W(1:10,:);
%     E = E(1:10,:);
%     E = [E; -0.9213 -1];
%     W = [W; 0.1278];
%     G = [G; zeros(1,Nu)];
    
    %W_x =[];G_x=[];E_x=[];
    
    W = [W_u; W_x; W_r];% Wxx];
    G = [G_u ; G_x; G_r];% Gxx];
    E = [E_u ; E_x; E_r];% Exx];
    %G_x
%     W = W(1:17,:);
%     G = G(1:17,:);
%     E = E(1:17,:);

    S = E + G*inv(H)*F';

end
