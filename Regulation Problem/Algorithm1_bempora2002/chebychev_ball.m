function [xc , r, diagnostics] = chebychev_ball(A, b, G, W, S, H, F, Nu, Nstate)
%[xc , r, diagnostics] = chebychev_ball(A, b, G, W, S, H, F, Nu, Nstate)
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
    z = sdpvar(Nu,1,'full'); 
    
    LMI = [];
    for i = 1:size(A,1)
        LMI = [LMI; A(i,:)*xc + r*sqrt(sum(A(i,:).^2)) <= b(i)];
    end
    LMI = [LMI; G*z - S*xc <= W];

    options=sdpsettings;
    options.solver='sedumi';
    options.verbose = 0;
    diagnostics = optimize(LMI,-r,options);
    
    xc = double(xc)
    r = double(r);

end

