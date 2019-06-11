function [ nodes_struct ] = nodes_to_struct(Nodes)
%[ nodes_struct ] = nodes_to_struct(Nodes)
%
%Convert cell array with the BST to struct data type
%Inputs:
%       Nodes - a cell array with all the nodes from the BST
%
%Outputs:
%       nodes_struct - struct array with six fields: A b left_node
%       right_node parent_node Region
%
max_size_region = 1;
number_state = size(Nodes{1,1},2);
for i = 1:size(Nodes,1)
    if size(Nodes{i,4},1) > max_size_region
       max_size_region = size(Nodes{i,4},1);
    end
end
max_size_region
for i = 1:size(Nodes,1)
    %nodes_struct(i).A = [Nodes{i,1}; zeros(max_size-(size(Nodes{i,1},1)),size(Nodes{i,1},2))];
    nodes_struct(i).A = [Nodes{i,1}];   
    if(isempty(nodes_struct(i).A))
        nodes_struct(i).A = zeros(1,number_state);
    end
    
    %nodes_struct(i).b = [Nodes{i,2}; zeros(max_size-(size(Nodes{i,1},1)),1)];
    nodes_struct(i).b = Nodes{i,2};
    if(isempty(nodes_struct(i).b))
        nodes_struct(i).b = 0;
    end
    
    nodes_struct(i).left_node = Nodes{i,5};
    if(isempty(nodes_struct(i).left_node))
        nodes_struct(i).left_node = 0;
    end
    
    nodes_struct(i).right_node = Nodes{i,6};
    if(isempty(nodes_struct(i).right_node))
        nodes_struct(i).right_node = 0;
    end
    
    nodes_struct(i).parent_node = Nodes{i,7};
    if(isempty(nodes_struct(i).parent_node))
        nodes_struct(i).parent_node = 0;
    end
    
    %nodes_struct(i).Region = Nodes{i,4};
    nodes_struct(i).Region = [Nodes{i,4}; zeros(max_size_region-(size(Nodes{i,4},1)),size(Nodes{i,4},2))];
    if(isempty(nodes_struct(i).Region))
        nodes_struct(i).Region = zeros(max_size_region,1);
    end
    
end

