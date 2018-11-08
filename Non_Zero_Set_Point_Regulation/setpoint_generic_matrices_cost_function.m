function [H, F, Sx, Su, Qlinha, Rlinha, Clinha] = setpoint_generic_matrices_cost_function(A, B, C, Q, R, Nstate, Ncontrol, Nout, Ny, Nu)
%[H, F, Sx, Su] = setpoint_generic_matrices_cost_function(A, B, C, Q, R, Nstate, Ncontrol, Ny, Nu)
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

    Su = zeros((Ny+1)*Nstate,Nu);
    for j = 1:Nu
        for i = 1:(Ny+1)
            if (i-j-1)>0     
                Su((Nstate*(i-1)+1):(Nstate*i),(Ncontrol*(j-1)+1):(Ncontrol*j)) =  [A^(i-j-1)*B];
            elseif (i-j-1) == 0
               Su((Nstate*(i-1)+1):(Nstate*(i)),(Ncontrol*(j-1)+1):(Ncontrol*j)) = [B];
            end
        end
    end
    
    Qlinha = Q;
    Rlinha = R;
    Clinha = C;
    I_ref = eye(Nout);
    
    for i = 1:Ny
        Qlinha = blkdiag(Qlinha,Q);
        Clinha = blkdiag(Clinha,C);
        I_ref = [I_ref; eye(Nout)];
    end
    
    for i = 1:(Nu-1)
        Rlinha = blkdiag(Rlinha,R);
    end
    
     H = Su'*Clinha'*Qlinha*Clinha*Su + Rlinha;
%      F = [Sx'*Clinha'*Qlinha*Clinha*Su;
%          -ones(Nout*(Ny+1),Nout)'*Qlinha*Clinha*Su];
    F = [Sx'*Clinha'*Qlinha*Clinha*Su;
          -I_ref'*Qlinha*Clinha*Su];
end

