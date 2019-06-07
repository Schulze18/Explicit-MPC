function [A_new, b_new, type_new, origem_new] = remove_redundant_constraints(A, b, type_new, origem_new, Nu, Nstate)
%[A_new, b_new] = remove_redundant_constraints(A, b, type_new, origem_new, Nu, Nstate)
%
%Remove the redundant constraints of a polyehdral defined by Ax<=b.
%Inputs:
%       A, b - matrices that may contain redundant constraints
%
%       Nu - control horizon
%
%       Nstate - number of states
%
%       type_new, origem_new - type of the original index that generated Ax<=b
%
%       origem_new -  original constraints that generated the hyperplane
%
%Outputs:
%       A_new, b_new - matrices without redundant constraints
%
%       type_new, origem_new - type and contraints from the reduced set of
%       contraints
%
%Algoritm based on the Lecture Notes "Polyhedral Computation - Spring 2014" from K.
%Fukuda.
    index = [];
      
    for i=1:size(A,1)
        x = sdpvar(Nstate,1,'full');
        LMI = [];
        for j=1:size(A,1)
            if j ~= i
                LMI = [LMI; A(j,:)*x <= b(j)];
            end
        end
        LMI = [LMI, A(i,:)*x <= (b(i)+1)];
        objetivo = A(i,:)*x;
        options = sdpsettings;
        options.solver = 'sedumi';
        options.verbose = 0;
        optimize(LMI,-objetivo,options);
        if(double(objetivo) <= b(i))
            index = [index, i];
        end
    end
%     A_new = [];
%     b_new = [];
%     for i = 1:size(A,1)
%         if i
%         A_new(i,:) = A(i,:);
%         b_new(i,:) = b(i,:);
%     
%     end
    
    A_new = A;
    b_new = b;
    index;
    for k = 1:size(index,2)
        A_new(index(k)-k+1,:) = [];
        b_new(index(k)-k+1,:) = [];
        type_new(index(k)-k+1,:) = [];
        origem_new(index(k)-k+1,:) = [];
    end

end

