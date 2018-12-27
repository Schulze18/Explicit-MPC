function [ result ] = verify_ineq_in_region(Region, ineq)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    % result = 0 - not in the region definition
    % result = 1 - <= sign definition
    % result = 2 - >= sign definition
    
    result = 0;
    A_region = Region{1,1};
    b_region = Region{1,2};

    A_ineq = ineq{1,1};
    b_ineq = ineq{1,2};

    for i = 1:size(A_region,1)
        test_matrix = [A_region(i,:) b_region(i,:); A_ineq b_ineq]; 
        rank_test = rank(test_matrix);
                
        sign_test = sum(test_matrix(1,:))/sum(test_matrix(2,:));
    
        if rank_test == 1 && sign_test > 0
            result = 1;
            break
        elseif rank_test == 1 && sign_test < 0
            result = 2;
            break
        end
   
    end
    
end

