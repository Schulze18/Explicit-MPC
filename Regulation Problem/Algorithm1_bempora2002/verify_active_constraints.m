function [G_tio, W_tio, S_tio] = verify_active_constraints(G, W, S, x0, z0, tol)
%[G_tio, W_tio, S_tio] = verify_active_constraints(G, W, S, x0, z0, tol)
%
%Determine the set of active constraints at given points x0 and z0
%Inputs:
%       G, W and S - from the matrix inequality G*z <= W + S*x(t)
%
%       x0 - feasible point inside the polyhedral set 
%
%       z0 - optimal solution of the mp-QP problem
%
%       tol - acceptable tolerance in the equation G_tio(i)*z0 = W + S*x0
%
%Outputs:
%       G_tio, W_tio and S_tio - rows of G, W and S corresponding to the active constraints
%
%Algoritm based on the paper "The explicit linear quadratic regulator for
%constrained systems" by A. Bemporad, M. Morari, V. Dua, and E. Pistikopoulos. 

index = [];
    for i = 1:length(W)
        %(G(i,:)*z0 - W(i,:) - S(i,:)*x0)
        if ((G(i,:)*z0 - W(i,:) - S(i,:)*x0 < tol) && (double(G(i,:)*z0 - W(i,:) - S(i,:)*x0)> -tol))
            index = [index , i]
        end
    end
    G_tio = [];
    S_tio = [];
    W_tio = [];
    if (length(index)>0)
        for i = 1:length(index)
            G_tio = [G_tio; G(index(i),:)];
            S_tio = [S_tio; S(index(i),:)];
            W_tio = [W_tio; W(index(i),:)];
        end
    end
end

