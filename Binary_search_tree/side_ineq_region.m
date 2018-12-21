function [result, diagnostics1, diagnostics2] = side_ineq_region(ineq,Region)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    A_ineq = ineq{1,1};
    b_ineq = ineq{1,2};
    result = 0;
    
    tol = 1e-5;
    
    %Check + sign
    x = sdpvar(size(A_ineq,2),1,'full');

    objective = A_ineq*x - b_ineq;
    options = sdpsettings;

    options.solver='sedumi';
    options.verbose = 0;
    options.cachesolvers = 1;
    
    A_region = Region{1,1};
    b_region = Region{1,2};

    LMI = [];
    for i = 1:size(A_region,1)
        LMI = [LMI; A_region(i,:)*x <= b_region(i,:) - tol];
    end

    diagnostics1 = optimize(LMI,objective,options);

    if double(objective) <= -tol;
        result = result + 1; 
    end
    
    %Check - sign
    x = sdpvar(size(A_ineq,2),1,'full');
    objective2 = -(A_ineq*x - b_ineq);
    LMI = [];
    for i = 1:size(A_region,1)
        LMI = [LMI; A_region(i,:)*x <= b_region(i,:)-tol];
    end
    
    diagnostics2 = optimize(LMI,objective2,options);
    
    if double(objective2) <= -tol
        result = result + 2; 
    end

end

