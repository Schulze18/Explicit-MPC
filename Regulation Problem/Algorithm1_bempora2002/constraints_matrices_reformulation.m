function [G, W, E, S] = constraints_matrices_reformulation(B, H, F, Nu, Umax, Umin, Xmax, Xmin)
%UNTITLED3 Summary of this function goes here
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
    
    num_state = size(B,1)         %Number of elements in the state vector
    num_control = size(B,2)       %Number of control inputs 
    
    %U constraints
    if isempty(Umax) == 0
        E_u = zeros(2*num_control*Nu,num_state);        
        W_u(1:num_control,1) = Umax;
        W_u((1+num_control):(2*num_control),1) = -Umin;
        G_u(1:num_control,1) = ones(size(Umax,2),1);
        G_u((1+num_control):(2*num_control),1) = -ones(size(Umin,2),1);  
        for i = 1:(Nu-1)
            G_u = blkdiag(G_u,G_u);
            W_u = [W_u; W_u];
        end
    end
    
    %X constraints
    if isempty(Xmax) == 0
        G_x = zeros(2*num_state,Nu);
        E_x = -eye(num_state);
        E_x = [E_x; eye(num_state)];
        W_x = Xmax';
        W_x = [W_x; (-Xmin)'];
    end
    %E_x
    %E_u
    W = [W_u; W_x];
    %G = [G_u zeros(num_state - Nu) ; G_x zeros(Nu - num_state)];
    G = [G_u ; G_x];
    E = [E_u ; E_x];
    %E = [E_u zeros(2*num_state - 2*num_control*Nu) ; E_x zeros(2*num_control*Nu - 2*num_state)];
    S = E + G*inv(H)*F';
    
end

