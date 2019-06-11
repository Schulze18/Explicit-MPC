function [ index_ineq ] = define_inequation_node(nodes, index_node, set_ineq, branch_index, controls)
%[ index_ineq ] = define_inequation_node(nodes, index_node, set_ineq, branch_index, controls)
%Define the inequation to be testes in the node
%
%Inputs:
%       nodes - cell array with the BST determined until now
%
%       index_node - index in the cell array from the evaluated node
%
%       set_ineq - set of all the inequations
%
%       branch_index - sequente of inequations to be "tested" in the branch
%       until the evaluated node
%
%       controls - list of all the control laws       
%
%Outputs:
%       index_ineq - Index of the inequation in the set of all the
%       inequations
%
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
    
    %If there is no new inequation form the regions definition, use only
    %new inequations.
    if isempty(set_new_index_ineq) == 1
     %   set_new_index_ineq = [];
        for i = 1:size(set_ineq,1)
            if ismember(i,branch_index) == 0
                set_new_index_ineq = [set_new_index_ineq; i];
            end
        end
    end
    
    %Among the set of index, find the one that define more control laws
    number_controls_less_equal = zeros(size(set_new_index_ineq,1),1);
    number_controls_greater_equal = zeros(size(set_new_index_ineq,1),1);
    num_max = 0;
    index_max = 0; 
    for i = 1:size(set_new_index_ineq,1)
        for j = 1:size(controls,1)
            if sum(ismember([branch_index; set_new_index_ineq(i)], controls{j,4})) == size([branch_index; set_new_index_ineq(i)],1)
                number_controls_less_equal(i) = number_controls_less_equal(i) + 1;
                
                if number_controls_less_equal(i) >  num_max
                    num_max = number_controls_less_equal(i);
                    index_max = set_new_index_ineq(i);
                end
            end
            if sum(ismember([branch_index; set_new_index_ineq(i)], controls{j,6})) == size([branch_index; set_new_index_ineq(i)],1)
                number_controls_greater_equal(i) = number_controls_greater_equal(i) + 1;
                
                 if number_controls_greater_equal(i) >  num_max
                    num_max = number_controls_greater_equal(i);
                    index_max = set_new_index_ineq(i);
                end
                
            end             
        end
    end
    
    
    
    if index_max > 0
        index_ineq = index_max;
    else
        disp('deu ruimm')
        index_ineq = set_new_index_ineq(1);     
    end
end

