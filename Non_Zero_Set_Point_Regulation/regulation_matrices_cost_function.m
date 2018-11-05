function [H, F, Sx, Su, Syx, Syu] = regulation_matrices_cost_function(A, B, C, P, Q, R, Ny, Nu)
%[H, F] = matrices_cost_function_reformulation(A, B, P, Q, R, Ny, Nu)
%
%Return de matrices H and F from the reformulated cost function with U to
%be opmitze.
%Inputs:
%       A,b - matrices from the state-space equation x[t+1] = A*x[t] + B*u[t]
%
%       P, Q, R - weighting matrices from the cost function:
%                 J(U,x[t]) = xNy'*P*xNy + sum(k=0,...,Ny-1){xk'*Q*xk + uk'*R*uk}
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

    Sx = eye(length(A));
    %Syx = C*eye(length(A));
    for i = 1:Ny
        Sx = [Sx; A^i];
        %Syx = [Syx; C*A^i];
    end

    Su = zeros((Ny+1)*length(B),Nu);
    %Syu = zeros((Ny+1),Nu);
    for j = 1:Nu
        for i = 1:(Ny+1)
            if (i-j-1)>0
                Su((length(B)*(i-1)+1):(length(B)*(i)),j) =  [A^(i-j-1)*B];
                %Syu(i,j) =  [C*A^(i-j-1)*B];
            elseif (i-j-1) == 0
                Su((length(B)*(i-1)+1):(length(B)*(i)),j) = [B];
                %Syu(i,j) = [C*B];
            end
        end
    end

%     Qlinha = Q*eye(Ny);
    Qlinha = Q;
    Clinha = C;
    for i = 1:(Ny-1)
        Qlinha = blkdiag(Qlinha,Q);
        Clinha = blkdiag(Clinha,C);
    end
    Clinha = blkdiag(Clinha,C);
    Qlinha = blkdiag(Qlinha,P);
    Rlinha = R*eye(Nu);
   
%     H = (Syu'*Qlinha*Syu + Rlinha);
%     F = (Syx'*Qlinha*Syu);
%     F = [F; -ones(1,Ny+1)*Qlinha*Syu];

      H = (Su'*Clinha'*Qlinha*Clinha*Su + Rlinha);
      F = (Sx'*Clinha'*Qlinha*Clinha*Su);
      F = [F; -ones(1,Ny+1)*Qlinha*Clinha*Su];
      Syu = 0;
      Syx = 0;
end

