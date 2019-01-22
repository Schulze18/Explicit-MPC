function [nodes_out, list_depth, number_leafs, max_depth] = count_depth_leafs(nodes)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    list_depth = {};
    nodes_out = nodes;
    nodes_out{1,9} = 0;
    max_depth = 0;
    number_leafs = 0;
    for i = 2:size(nodes_out,1)
        index_parent = nodes_out{i,7};
        nodes_out{i,9} = nodes_out{index_parent,9} + 1; 
        
        if nodes_out{i,9} > max_depth
            max_depth = nodes_out{i,9};
            list_depth{max_depth,1} = 0;
        end
        
        if isempty(nodes_out{i,1}) == 1
            number_leafs = number_leafs + 1;
            
            if  list_depth{nodes_out{i,9},1} == 0
                list_depth{nodes_out{i,9},1} = 1;
                list_depth{nodes_out{i,9},2} = i;
            else
                list_depth{nodes_out{i,9},1} = list_depth{nodes_out{i,9},1} + 1;
                list_depth{nodes_out{i,9},2} = [list_depth{nodes_out{i,9},2}; i];
            end
        end
    end
    
    for i = 1:size(list_depth,1)
        list_depth{i,3} = list_depth{i,1}/number_leafs*100;
    end
    
    
end

