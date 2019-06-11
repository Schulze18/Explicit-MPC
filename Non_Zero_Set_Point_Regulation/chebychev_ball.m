function [xc , r, diagnostics] = chebychev_ball(A, b, G, W, S, H, F, Nstate, Ncontrol, Nout, Ny, Nu, options)
%[xc , r, diagnostics] = chebychev_ball(A, b, G, W, S, H, F, Nstate, Ncontrol, Nout, Ny, Nu, options)
%
%Return the center of the largest possible ball that can be placed inside
%the region defined by Ax<=b (Chebyshev center) and that Vz(x) is feasible.
%Inputs:
%       A, b - matrices that define the polyhedral Ax <= b
%
%       G, W, S, H and F - from the cost function: 
%                          Vz(x) = 0.5*z'*H*z 
%                                 through z 
%                                 subject to G*z <= W + S*x(t)
%       
%       Nstate, Ncontrol, Nout - number of states, control actions and
%       outputs of the system
%
%       Ny, Nu - prediction and control horizon 
%
%       options - solver options for yalmip 
%
%Outputs:
%       xc - Chebyshev center
%
%       r - radius of the Chebyshev ball
%
%       diagnostics - returns the "optimize" function status
%
%Algoritm based on the paper "The explicit linear quadratic regulator for
%constrained systems" by A. Bemporad, M. Morari, V. Dua, and E. Pistikopoulos. 
   
    xc = sdpvar(Nstate,1,'full');
    r = sdpvar(1);
    z = sdpvar(Nu*Ncontrol,1,'full'); 
    
    LMI = [];
    for i = 1:size(A,1)
        LMI = [LMI; A(i,:)*xc + r*sqrt(sum(A(i,:).^2)) <= b(i)];
    end
    LMI = [LMI; G*z - S*xc <= W];

% %     options=sdpsettings;
%     options.solver='sedumi';
% %     options.solver='sdpt3';
% %     options.verbose = 0;
% %     options.cachesolvers = 1;
    
    options.sdpt3.maxit = 20;
    options.sdpt3.steptol = 1.0000e-08;
    options.sdpt3.gaptol = 5.000e-8;
    
    diagnostics = optimize(LMI,-r,options);
%     check(LMI)
    
    xc = double(xc);
    r = double(r);

end

