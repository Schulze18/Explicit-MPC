function  [G, W, E, S, num_Gu] = setpoint_generic_constraints(Sx, Su, Clinha, H, F, Nstate, Ncontrol, Nout, Nref, Ny, Nu, U_max, U_min, Ref_max, Ref_min, X_max, X_min, Y_max, Y_min)
%function  [G_u, W_u, E_u, G_u_max, W_u_max, E_u_max, G_r, W_r, E_r,num_Gu] = setpoint_generic_constraints(Sx, Su, Clinha, H, F, Nstate, Ncontrol, Nout, Nref, Ny, Nu, U_max, U_min, X_max, X_min, Ref_max,Ref_min)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    G_u = [];
    G_x = [];
    G_r = [];
    G_y = [];
    W_u = [];
    W_x = [];
    W_r = [];
    W_y = [];
    E_u = [];
    E_x = [];
    E_r = [];
    E_y = [];
    S = [];
    
    if isempty(U_max) == 0
        G_u_max = [];
        W_u_max = [];
        E_u_max = [];
        %Constraints in a specific control actions
        for i = 1:Ncontrol
            if(U_max(i) ~= Inf)
                G_u_max_i = zeros(1,Ncontrol);
                G_u_max_i(1,i) = 1;
                G_u_max = [G_u_max; G_u_max_i];
                
                W_u_max_i = U_max(i);
                W_u_max = [W_u_max; W_u_max_i];
                
                E_u_max_i = zeros(1,Nstate + Nref);
                E_u_max = [E_u_max; E_u_max_i];
            end
        end
        
        G_u_nu_max = [];
        W_u_nu_max = [];
        E_u_nu_max = [];
        for i = 1:Nu
            G_u_nu_max = blkdiag( G_u_nu_max,G_u_max);
            W_u_nu_max = [W_u_nu_max; W_u_max];
            E_u_nu_max = [E_u_nu_max; E_u_max];
        end
        G_u = [G_u; G_u_nu_max];
        W_u = [W_u; W_u_nu_max];
        E_u = [E_u; E_u_nu_max];
        
%         G_u = G_u_max;
%         W_u = W_u_max;
%         E_u = E_u_max;

%         G_u = [eye(Nu*Ncontrol);
%               -eye(Nu*Ncontrol)];
%           
%         W_u = eye(Ncontrol)*U_max;
%         for i = 1:(Nu-1)
%             W_u = [W_u; eye(Ncontrol)*U_max];
%         end
%         for i = 1:Nu
%             W_u = [W_u; -eye(Ncontrol)*U_min];
%         end
%         
%         E_u = zeros(2*Ncontrol*Nu,Nstate + Nout);
    end
    
    if isempty(U_min) == 0
        G_u_min = [];
        W_u_min = [];
        E_u_min = [];
        for i = 1:Ncontrol
            if(U_min(i) ~= -Inf)
                G_u_min_i = zeros(1,Ncontrol);
                G_u_min_i(1,i) = -1;
                G_u_min = [G_u_min; G_u_min_i];
                
                W_u_min_i = -U_min(i);
                W_u_min = [W_u_min; W_u_min_i];
                
                E_u_min_i = zeros(1,Nstate + Nref);
                E_u_min = [E_u_min; E_u_min_i];
            end
        end
        
        G_u_nu_min = [];
        W_u_nu_min = [];
        E_u_nu_min = [];
        for i = 1:Nu
            G_u_nu_min = blkdiag(G_u_nu_min,G_u_min);
            W_u_nu_min = [W_u_nu_min; W_u_min];
            E_u_nu_min = [E_u_nu_min; E_u_min];
        end
        G_u = [G_u; G_u_nu_min];
        W_u = [W_u; W_u_nu_min];
        E_u = [E_u; E_u_nu_min];

    end
        
    if isempty(Ref_max) == 0
        G_r_max = [];
        W_r_max = [];
        E_r_max = []; 
        for i = 1:Nref
                if(Ref_max(i) ~= Inf)
                    G_r_max_i = zeros(1,Ncontrol*Nu);
                    G_r_max = [G_r_max; G_r_max_i];
        
                    W_r_max_i = Ref_max(i);
                    W_r_max = [W_r_max; W_r_max_i];
                    
                    E_r_max_i = zeros(1,Nstate + Nref);
                    E_r_max_i(1,Nstate+i) = -1;
                    E_r_max = [E_r_max; E_r_max_i];
                end
        end
       
        G_r = [G_r; G_r_max];
        W_r = [W_r; W_r_max];
        E_r = [E_r; E_r_max];    
    end

    
    if isempty(Ref_min) == 0
        G_r_min = [];
        W_r_min = [];
        E_r_min = [];
        for i = 1:Nout
                if(Ref_min(i) ~= -Inf)
                    G_r_min_i = zeros(1,Ncontrol*Nu);
                    G_r_min = [G_r_min; G_r_min_i];
        
                    W_r_min_i = -Ref_min(i);
                    W_r_min = [W_r_min; W_r_min_i];
                    
                    E_r_min_i = zeros(1,Nstate + Nref);
                    E_r_min_i(1,Nstate+i) = 1;
                    E_r_min = [E_r_min; E_r_min_i];
                end
        end
 
        G_r = [G_r; G_r_min];
        W_r = [W_r; W_r_min];
        E_r = [E_r; E_r_min];    
    end
    
    
    if isempty(X_max) == 0
        G_x_max = [];
        W_x_max = [];
        E_x_max = [];
        
        for i = 1:(Ny+1)
            for j = 1:Nstate
                if (X_max(j) ~= Inf)
                    G_x_max = [G_x_max; Su(j+(i-1)*Nstate,:)];
                    W_x_max = [W_x_max; X_max(j)];
                    E_x_max = [E_x_max; -Sx(j+(i-1)*Nstate,:) zeros(1,Nref)];
                end
            end
        end
        G_x = [G_x; G_x_max];
        W_x = [W_x; W_x_max];
        E_x = [E_x; E_x_max];
    end
    
    if isempty(X_min) == 0
        G_x_min = [];
        W_x_min = [];
        E_x_min = [];
        
        for i = 1:(Ny+1)
            for j = 1:Nstate
                if (X_min(j) ~= -Inf)
                    G_x_min = [G_x_min; -Su(j+(i-1)*Nstate,:)];
                    W_x_min = [W_x_min; -X_min(j)];
                    E_x_min = [E_x_min; Sx(j+(i-1)*Nstate,:) zeros(1,Nref)];
                end
            end
        end
        G_x = [G_x; G_x_min];
        W_x = [W_x; W_x_min];
        E_x = [E_x; E_x_min];
        
    end
    
%     W = W_x;
%     G = G_x;
%     E = E_x;
%     S = 1;

    if isempty(Y_max) == 0
        G_y_max = [];
        W_y_max = [];
        E_y_max = [];
        Suy = Clinha*Su;
        Sxy = Clinha*Sx;
        %for i = 1:(Ny+1)
        for i = 2:(Ny+1)
            for j = 1:Nout
                if (Y_max(j) ~= Inf)
                    G_y_max = [G_y_max; Suy(j+(i-1)*Nout,:)];
                    W_y_max = [W_y_max; Y_max(j)];
                    E_y_max = [E_y_max; -Sxy(j+(i-1)*Nout,:) zeros(1,Nref)];
                end
            end
         end
        
        G_y = [G_y; G_y_max];
        W_y = [W_y; W_y_max];
        E_y = [E_y; E_y_max];
        
    end
    
    if isempty(Y_min) == 0
        G_y_min = [];
        W_y_min = [];
        E_y_min = [];
        Suy = Clinha*Su;
        Sxy = Clinha*Sx;
        %for i = 1:(Ny+1)
        for i = 2:(Ny+1)
            for j = 1:Nout
                if (Y_min(j) ~= -Inf)
                    G_y_min = [G_y_min; -Suy(j+(i-1)*Nout,:)];
                    W_y_min = [W_y_min; -Y_min(j)];
                    E_y_min = [E_y_min; Sxy(j+(i-1)*Nout,:) zeros(1,Nref)];
                end
            end
         end
        
        G_y = [G_y; G_y_min];
        W_y = [W_y; W_y_min];
        E_y = [E_y; E_y_min];
        
    end
    

    num_Gu = size(G_u,1) + size(G_r,1);
    W = [W_u; W_r; W_x; W_y];
    G = [G_u; G_r; G_x; G_y];
    E = [E_u; E_r; E_x; E_y];

    S = E + G*inv(H)*F';

end

