function [z0, diagnostics] = optimal_z_mp_QP(G, W, S, H, F, x0, Nstate, Ncontrol, Nout, Ny, Nu, options)
%[z0, diagnostics] = optimal_z_mp_QP(G, W, S, H, F, x0, Nstate, Ncontrol, Nout, Ny, Nu, options)
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
%       Nstate, Ncontrol, Nout - number of states, control actions and
%       outputs of the system
%
%       Ny, Nu - prediction and control horizon 
%
%       options - solver options for yalmip 
% 
%Outputs:
%       z0 - optimal solution
%
%       diagnostics - returns the "optimize" function status
%
%Algoritm based on the paper "The explicit linear quadratic regulator for
%constrained systems" by A. Bemporad, M. Morari, V. Dua, and E. Pistikopoulos. 
    U = sdpvar(Nu*Nout,1,'full'); 
    z = U + inv(H)*F'*x0; 

    LMI = [];
    %LMI = [LMI, -G*z + W + S*x0 >= 0 ];
     LMI = [LMI, G*z <= W + S*x0 ];
    objetivo = 0.5*z'*H*z;
% %     options = sdpsettings;
% %     options.solver = 'sedumi';
%     options.verbose = 0;
    
% %     options.solver='sdpt3';
% %     options.verbose = 0;
% %     options.cachesolvers = 1;
    
    options.sdpt3.maxit = 100;
    options.sdpt3.steptol = 1.0000e-05;
    options.sdpt3.gaptol = 5.000e-5;
    
    diagnostics = optimize(LMI,objetivo,options);
    z0 = double(z);

end

