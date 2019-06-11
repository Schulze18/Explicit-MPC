function  [G, W, E, S, num_Gu] = setpoint_deltaU_generic_constraints(Sx, Su, Sdu, Clinha, H, F, Nstate, Ncontrol, Nout, Nref, Ny, Nu, deltaU_max, deltaU_min, U_max, U_min, Ref_max, Ref_min, X_max, X_min, Y_max, Y_min)
%[G, W, E, S, num_Gu] = setpoint_deltaU_generic_constraints(Sx, Su, Sdu, Clinha, H, F, Nstate, Ncontrol, Nout, Nref, Ny, Nu, deltaU_max, deltaU_min, U_max, U_min, Ref_max, Ref_min, X_max, X_min, Y_max, Y_min)
%
%Return de matrices H and F from the reformulated cost function for setpoint in a non-zero point regulation with Delta U to
%be optimize
%Inputs:
%       H, F - from the cost function:
%              V(x(t)) = 0.5*x'*Y*x + min {0.5*U'*H*U + x'*F*U}
%                                     through U 
%                                     subject to G*U <= W + E*x
%
%       Sx, Su, Sdu, Clinha - matrices from the state space over the prediction and control
%       horizon: X = Sx*x[t] + Su*u0 + Sdu*Delta_U; Y = Clinha*X
%               
%       Nstate, Ncontrol, Nout, Nref - number of states, control actions,
%       outputs and references from the system
%
%       Ny, Ny - prediction and control horizon
%
%       deltaU_max, deltaU_min, U_max, U_min, Ref_max, Ref_min, X_max,
%       X_min, Y_max, Y_min - array with the max and min values
%
%Outpus:
%        G, W, E and S - from the matrix inequalities G*z <= W + S*x(t) and GU <= W + Ex(t) 
%
%        num_Gu - number of rows from G related to constraints in control
%        action U


    G_du = [];
    G_u = [];
    G_x = [];
    G_r = [];
    G_y = [];
    W_du = [];
    W_u = [];
    W_x = [];
    W_r = [];
    W_y = [];
    E_du = [];
    E_u = [];
    E_x = [];
    E_r = [];
    E_y = [];
    S = [];
    
    if isempty(deltaU_max) == 0
        G_du_max = [];
        W_du_max = [];
        E_du_max = [];
        %Constraints in a specific control rate actions
        for i = 1:Ncontrol
            if(deltaU_max(i) ~= Inf)
                G_du_max_i = zeros(1,Ncontrol);
                G_du_max_i(1,i) = 1;
                G_du_max = [G_du_max; G_du_max_i];
                
                W_du_max_i = deltaU_max(i);
                W_du_max = [W_du_max; W_du_max_i];
                
                E_du_max_i = zeros(1,Nstate + Ncontrol + Nref);
                E_du_max = [E_du_max; E_du_max_i];
            end
        end
        
        G_du_nu_max = [];
        W_du_nu_max = [];
        E_du_nu_max = [];
        for i = 1:Nu
            G_du_nu_max = blkdiag(G_du_nu_max,G_du_max);
            W_du_nu_max = [W_du_nu_max; W_du_max];
            E_du_nu_max = [E_du_nu_max; E_du_max];
        end
        G_du = [G_du; G_du_nu_max];
        W_du = [W_du; W_du_nu_max];
        E_du = [E_du; E_du_nu_max];
        
    end
    
    
     if isempty(deltaU_min) == 0
        G_du_min = [];
        W_du_min = [];
        E_du_min = [];
        for i = 1:Ncontrol
            if(deltaU_min(i) ~= -Inf)
                G_du_min_i = zeros(1,Ncontrol);
                G_du_min_i(1,i) = -1;
                G_du_min = [G_du_min; G_du_min_i];
                
                W_du_min_i = -deltaU_min(i);
                W_du_min = [W_du_min; W_du_min_i];
                
                E_du_min_i = zeros(1,Nstate + Ncontrol + Nref);
                E_du_min = [E_du_min; E_du_min_i];
            end
        end
        
        G_du_nu_min = [];
        W_du_nu_min = [];
        E_du_nu_min = [];
        for i = 1:Nu
            G_du_nu_min = blkdiag(G_du_nu_min,G_du_min);
            W_du_nu_min = [W_du_nu_min; W_du_min];
            E_du_nu_min = [E_du_nu_min; E_du_min];
        end
        G_du = [G_du; G_du_nu_min];
        W_du = [W_du; W_du_nu_min];
        E_du = [E_du; E_du_nu_min];

     end
    
    
     if isempty(U_max) == 0
        G_u_max = [];
        W_u_max = [];
        E_u_max = [];
        T_du = [];
        %Constraints in a specific control actions
        T_du_temp_i = [];
        for i = 1:Ncontrol
            if(U_max(i) ~= Inf)
                T_du_temp_i = [T_du_temp_i; zeros(1,Ncontrol)];
                T_du_temp_i(end,i) = 1;
                                      
                W_u_max_i = U_max(i);
                W_u_max = [W_u_max; W_u_max_i];
                
                E_u_max_temp = zeros(1,Ncontrol);
                E_u_max_temp(1,i) = -1;
                E_u_max_i = [zeros(1,Nstate) E_u_max_temp zeros(1,Nref)];
                E_u_max = [E_u_max; E_u_max_i];
            end
        end
        
        for i = 1:Nu
            T_du_temp = [];
            for j = 1:Nu
                if j <= i
                    T_du_temp = [T_du_temp T_du_temp_i];
                else
                    T_du_temp = [T_du_temp zeros(size(T_du_temp_i,1),Ncontrol)];
                end
            end
            T_du = [T_du; T_du_temp];
        end
        
        G_u_nu_max = T_du;
        W_u_nu_max = [];
        E_u_nu_max = [];
        for i = 1:Nu
            W_u_nu_max = [W_u_nu_max; W_u_max];
            E_u_nu_max = [E_u_nu_max; E_u_max];
        end
        G_u = [G_u; G_u_nu_max];
        W_u = [W_u; W_u_nu_max];
        E_u = [E_u; E_u_nu_max];
        
     end
    
    if isempty(U_min) == 0
        G_u_min = [];
        W_u_min = [];
        E_u_min = [];
        T_du = [];
        %Constraints in a specific control actions
        T_du_temp_i = [];
        for i = 1:Ncontrol
            if(U_min(i) ~= -Inf)
                T_du_temp_i = [T_du_temp_i; zeros(1,Ncontrol)];
                T_du_temp_i(end,i) = -1;
                                      
                W_u_min_i = -U_min(i);
                W_u_min = [W_u_min; W_u_min_i];
                
                E_u_min_temp = zeros(1,Ncontrol);
                E_u_min_temp(1,i) = 1;
                E_u_min_i = [zeros(1,Nstate) E_u_min_temp zeros(1,Nref)];
                E_u_min = [E_u_min; E_u_min_i];
            end
        end
        
        for i = 1:Nu
            T_du_temp = [];
            for j = 1:Nu
                if j <= i
                    T_du_temp = [T_du_temp T_du_temp_i];
                else
                    T_du_temp = [T_du_temp zeros(size(T_du_temp_i,1),Ncontrol)];
                end
            end
            T_du = [T_du; T_du_temp];
        end
        
        G_u_nu_min = T_du;
        W_u_nu_min = [];
        E_u_nu_min = [];
        for i = 1:Nu
            W_u_nu_min = [W_u_nu_min; W_u_min];
            E_u_nu_min = [E_u_nu_min; E_u_min];
        end
        G_u = [G_u; G_u_nu_min];
        W_u = [W_u; W_u_nu_min];
        E_u = [E_u; E_u_nu_min];
        
    end 
     
    num_Gu = size(G_du,1) + size(G_u,1) + size(G_r,1);
    W = [W_du; W_u; W_r; W_x; W_y];
    G = [G_du; G_u; G_r; G_x; G_y];
    E = [E_du; E_u; E_r; E_x; E_y];

    S = E + G*inv(H)*F';
    
    
    
end

