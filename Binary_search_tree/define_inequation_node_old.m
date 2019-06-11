function [ index_ineq ] = define_inequation_node_old(nodes, index_node, set_ineq)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

    flag_new_ineq = 0;
    for i = 1:size(nodes{index_node,4},1)
        if flag_new_ineq == 1
                break
        end
        for j = 1:size(set_ineq,1)
            if flag_new_ineq == 1
                break
            end
            %Verify inequation "not used" that define a Region 
            for k = 1:size(set_ineq{j,3},1)
                if (set_ineq{j,3}(k,1) == nodes{index_node,4}(i,1)) && (ismember(j,nodes{index_node,3}) == 0)%j ~= last_index_ineq
                    index_ineq = j;
                    flag_new_ineq = 1;
                    break
                end
            end
        end
    end

end

