function [G_tio, W_tio, S_tio] = build_active_const(G, W, S, index)
%[G_tio, W_tio, S_tio] = build_active_const(G, W, S, index)
%
%Return the rows of G, W and S associated with the active constraints
%Inputs:
%       G, W, S - from the cost function: 
%                          Vz(x) = 0.5*z'*H*z 
%                                 through z 
%                                 subject to G*z <= W + S*x(t)
%
%       index - list of active constraints 
%
%Outputs:
%       G_tio, W_tio, S_tio - rows of G, W and S associated with the active constraints
%
%Algoritm based on the paper "The explicit linear quadratic regulator for
%constrained systems" by A. Bemporad, M. Morari, V. Dua, and E. Pistikopoulos. 
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

