function [G, W, E, S] = constraints_matrices_reformulation(A, B, H, F, Sx, Su, Ny, Nu, Umax, Umin, Xmax, Xmin)
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
    W_u = [];
    W_x = [];
    E_u = [];
    E_x = [];
    S = [];
    
    num_state = size(B,1);         %Number of elements in the state vector
    num_control = size(B,2);       %Number of control inputs 
    
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
        E_u = zeros(2*num_control*Nu,num_state); 
        G_u = eye(Nu*num_control);
        G_u = [G_u;-G_u];
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
        G_x = zeros(2*num_state,Nu);
        E_x = -eye(num_state);
        E_x = [E_x; eye(num_state)];
        W_x = Xmax';
        W_x = [W_x; (-Xmin)'];
        
        %%Faz mais sentido
        G_x = [Su; -Su];
        E_x = [-Sx; Sx];
        W_x = eye(num_state)*Xmax';
        for i = 1:(Ny)
            W_x = [W_x; eye(num_state)*Xmax'];
        end
        for i = 1:(Ny+1)
            W_x = [W_x; -eye(num_state)*Xmin'];
        end       
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

    vetor_reta = [1 1.1];
    reta_offset = 0.22;%0.1278
    Jx =  vetor_reta*eye(length(A));
    for i = 1:Ny
        Jx = [Jx; vetor_reta*A^i];
    end
    Ju = zeros((Ny+1),Nu);
    for j = 1:Nu
        for i = 1:(Ny+1)
            if (i-j-1)>0
                Ju((i-1+1):((i)),j) =  [vetor_reta*A^(i-j-1)*B];
            elseif (i-j-1) == 0
                Ju((i-1+1):i,j) = [vetor_reta*B];
            end
        end
    end
   
    Gxx = Ju;
    Exx = -Jx;
    Wxx = reta_offset*ones((Ny+1),1);
    
    %W_x =[];G_x=[];E_x=[];
    
    W = [W_u; W_x];% Wxx];
    G = [G_u ; G_x];% Gxx];
    E = [E_u ; E_x];% Exx];
    %G_x
%     W = W(1:17,:);
%     G = G(1:17,:);
%     E = E(1:17,:);

    S = E + G*inv(H)*F';

end
