function [H, F, Sx, Su, Sdu, T_du, Qlinha, Rulinha, Rdulinha, Clinha] = setpoint_deltaU_matrices_cost_function(A, B, C, Q, Ru, Rdu, Nstate, Ncontrol, Nout, Ny, Nu)
%[H, F, Sx, Su, Sdu, Qlinha, Rlinha, Clinha] = setpoint_deltaU_matrices_cost_function(A, B, C, Q, R, Nstate, Ncontrol, Nout, Ny, Nu)
%
%Return de matrices H and F from the reformulated cost function with U to
%be opmitze.
%Inputs:
%       A, B, C - matrices from the state-space equation x[t+1] = A*x[t] + B*u[t]
%
%       Q, R - weighting matrices from the cost function:
%                 J(U,x[t]) = sum(k=0,...,Ny){(Yref - Yk)'*Q*(Yref - Yk)} + sum(k=0,...,Nu-1){uk'*Q*uk}
%               
%       Nstate - number of states
%
%       Ncontrol - number of control actions        
%       
%       Nout - number of outputs        
%
%       Ny - output horizon
%
%       Nu - control horizon
%
%Outputs:
%       H, F - from the cost function:
%              V(x(t)) = 0.5*x'*Y*x + min {0.5*U'*H*U + x'*F*U}
%                                     through U 
%                                     subject to G*U <= W + E*x
%
%Algoritm based on the paper "The explicit linear quadratic regulator for
%constrained systems" by A. Bemporad, M. Morari, V. Dua, and E. Pistikopoulos. 
    
    Sx = eye(Nstate);
    for i = 1:Ny
        Sx = [Sx; A^i];
    end
   
    Su = zeros(Nstate,Ncontrol);
    for i = 2:(Ny+1)
        Su_temp = zeros(Nstate,Ncontrol);
        for j = 0:(i-2)
            Su_temp = Su_temp + (A^j)*B;
        end
        Su = [Su; Su_temp];
    end
    
        Sdu = zeros((Ny+1)*Nstate,Nu);
    for j = 1:Nu
        for i = 1:(Ny+1)
            if (i-j-1)>0
                Sdu_temp = zeros(Nstate,Ncontrol);
                for k = 0:(i-j-1)
                    Sdu_temp = Sdu_temp + (A^k)*B;
                end
                Sdu((Nstate*(i-1)+1):(Nstate*i),(Ncontrol*(j-1)+1):(Ncontrol*j)) =  [Sdu_temp];
            elseif (i-j-1) == 0
                Sdu((Nstate*(i-1)+1):(Nstate*(i)),(Ncontrol*(j-1)+1):(Ncontrol*j)) = [B];
            end
        end
    end
    
   
    Qlinha = Q;
    Rulinha = Ru;
    Rdulinha = Rdu;
    Clinha = C;
    I_ref = eye(Nout);
    I_u = eye(Ncontrol);
    
    for i = 1:Ny
        Qlinha = blkdiag(Qlinha,Q);
        Clinha = blkdiag(Clinha,C);
        I_ref = [I_ref; eye(Nout)];
    end
    
    for i = 1:(Nu-1)
        Rulinha = blkdiag(Rulinha,Ru);
        Rdulinha = blkdiag(Rdulinha,Rdu);
        I_u = [I_u; eye(Ncontrol)];
    end
    
    T_du = [];
    for i = 1:Nu
        T_du_temp = [];
        for j = 1:Nu
            if j <= i
                T_du_temp = [T_du_temp eye(Ncontrol)];
            else
                T_du_temp = [T_du_temp zeros(Ncontrol)];
            end
        end
        T_du = [T_du; T_du_temp];
    end
   
    H =  T_du'*Rulinha*T_du + Rdulinha + Sdu'*Clinha'*Qlinha*Clinha*Sdu;
    F = [Sx'*Clinha'*Qlinha*Clinha*Sdu;
         Su'*Clinha'*Qlinha*Clinha*Sdu + I_u'*Rulinha*T_du;
        -I_ref'*Qlinha*Clinha*Sdu];
    
end

