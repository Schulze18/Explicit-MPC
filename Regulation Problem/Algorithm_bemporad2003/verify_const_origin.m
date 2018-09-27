function [ origem ] = verify_const_origin(A, b, G, W, S, z0)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    INEQ_A = -S;
    INEQ_b = W - G*z0;
    origem = zeros(size(A,1),1);
    
    for i = 1:size(A,1)
        for j = 1:size(G,1)
            if A(i,1) == 0
                if (A(i,2)/INEQ_A(j,2) == b(i,1)/INEQ_b(j,1))
                    origem(i) = j;
                    break
                end
            else
            if (A(i,1)/INEQ_A(j,1) == b(i,1)/INEQ_b(j,1))
                 origem(i) = j;
                 break
            end
        end
        end
    end


end

