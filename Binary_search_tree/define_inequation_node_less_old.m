function [ index_ineq, num_max, flag_ineq_region ] = define_inequation_node_less_old(nodes, index_node, set_ineq, branch_index, controls)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

    %Find inequation that define the regions but were not used
    set_new_index_ineq = [];
    for i = 1:size(nodes{index_node,4},1)   
        for j = 1:size(set_ineq,1)
            %Verify inequation "not used" that define a Region 
            for k = 1:size(set_ineq{j,3},1)
                if (set_ineq{j,3}(k,1) == nodes{index_node,4}(i,1)) && (ismember(j,nodes{index_node,3}) == 0) && (ismember(j,set_new_index_ineq) == 0)  %j ~= last_index_ineq
                    set_new_index_ineq = [set_new_index_ineq; j];
                end
            end
        end
    end
    
    flag_ineq_region = 1;
    
    %If there is no new inequation form the regions definition, use just
    %new inequations.
    if isempty(set_new_index_ineq) == 1
        flag_ineq_region = 0;
     %   set_new_index_ineq = [];
        for i = 1:size(set_ineq,1)
            if ismember(i,branch_index) == 0
                set_new_index_ineq = [set_new_index_ineq; i];
            end
        end
    end
    
