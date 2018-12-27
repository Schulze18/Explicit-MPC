function [nodes_out, number_leafs, number_depth] = count_depth_leafs(nodes)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    nodes_out = nodes;
    nodes_out{1,8} = 0;
    number_depth = 0;
    number_leafs = 0;
    for i = 2:size(nodes_out,1)
        index_parent = nodes_out{i,7};
        nodes_out{i,8} = nodes_out{index_parent,8} + 1; 
        if nodes_out{i,8} > number_depth
            number_depth = nodes_out{i,8};
        end
        if isempty(nodes_out{i,1}) == 1
            number_leafs = number_leafs + 1;
        end
    end

end

