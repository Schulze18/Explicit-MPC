function [z0, diagnostics] = optimal_z_mp_QP(G, W, S, H, F, x0, Nu)
%[z0, diagnostics] = optimal_z_mp_QP(G, W, S, H, F, x0, Nu)
%
%Calculate the optmizal z to the mp-QP problem: 
%V(x) = min 0.5*z'*H*z , subject to Gz <= W +S*x(t)
%Inputs:
%       G, W, S, H and F - from the cost functions: 
%                          Vz(x) = 0.5*z'*H*z 
%                                  through z 
%                                  subject to G*z <= W + S*x(t)
%    
%       x0 - feasible point inside the polyhedral set 
%    
%       Nu - control horizon                       
% 
%Outputs:
%       z0 - optimal solution
%
%       diagnostics - returns the "optimize" function status
%
%Algoritm based on the paper "The explicit linear quadratic regulator for
%constrained systems" by A. Bemporad, M. Morari, V. Dua, and E. Pistikopoulos. 
    U = sdpvar(Nu,1,'full'); 
    x0
    z = U + inv(H)*F'*x0; 

    LMI = [];
    %LMI = [LMI, -G*z + W + S*x0 >= 0 ];
     LMI = [LMI, G*z <= W + S*x0 ];
    objetivo = 0.5*z'*H*z;
    options = sdpsettings;
    options.solver = 'sedumi';
    options.verbose = 0;
    
    diagnostics = optimize(LMI,objetivo,options);
    z0 = double(z);

end

