function [index_region, num_operations] = evaluate_region_BST(x,nodes)
%[index_region, num_operations] = evaluate_region_BST(x,nodes)
%   
%Find the index of the region evaluating all the BST
%Inputs:
%       x - value to be tested in each inequation
%
%       nodes - cell array with all the nodes from the BST
%
%Outputs:
%       index_region - index of the region from the explicit controller
%
%       num_operations - number of operations to find the region


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
%     disp('test')
    if isempty(nodes{index_node,4})
        index_region = 0;
    else
        index_region = nodes{index_node,4}(1,1);
    end
 end

