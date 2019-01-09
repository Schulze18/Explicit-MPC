function [index_region, num_operations] = evaluate_region_BST(x,nodes)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    index_node = 1;
    num_operations = 0;
     
    while isempty(nodes{index_node,5}) == 0
        num_operations = num_operations + 1;
        A_ineq = nodes{index_node,1};
        b_ineq = nodes{index_node,2};
        dj = A_ineq*x - b_ineq;
        if dj <= 0
            index_node = nodes{index_node,5};
        else
            index_node = nodes{index_node,6};
        end
    end
    disp('test')
    if isempty(nodes{index_node,4})
        index_region = 0;
    else
        index_region = nodes{index_node,4}(1,1);
    end
 end

