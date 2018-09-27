function [A_new, b_new] = remove_redundant_constraints(A, b, Nu, Nstate)
%[A_new, b_new] = remove_redundant_constraints(A, b, Nu, Nstate)
%
%Remove the redundant constraints of a polyehdral defined by Ax<=b.
%Inputs:
%       A, b - matrices that may contain redundant constraints
%
%Outputs:
%       A_new, b_new - matrices without redundant constraints
%
%Algoritm based on the paper "The explicit linear quadratic regulator for
%constrained systems" by A. Bemporad, M. Morari, V. Dua, and E. Pistikopoulos. 

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
    end

end

