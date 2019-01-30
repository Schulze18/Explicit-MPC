function [ index_ineq, num_min, flag_ineq_region ] = define_inequation_node_less(nodes, index_node, set_ineq, branch_index, controls, Regions, tol)
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
    
    %Among the set of index, find the one that the number of remaining
    %control laws are the smaller
    number_controls_less_equal = zeros(size(set_new_index_ineq,1),1);
    number_controls_greater_equal = zeros(size(set_new_index_ineq,1),1);
    
    
    %Less side
    for i = 1:size(set_new_index_ineq,1)
        set_test_index = [branch_index; set_new_index_ineq(i)];
        side_set_test_index = [nodes{index_node,8}; 1];
        set_new_regions = {}; %%%
        for j = 1:size(nodes{index_node,4},1)
            
            flag_new_region = 1;
            for k = 1:size(set_test_index,1)
                side_region = set_ineq{set_test_index(k,1),6}(nodes{index_node,4}(j,1),1);
                if (((side_set_test_index(k) == 1) && (side_region == 1 || side_region== 3)) == 0 && ((side_set_test_index(k) == 2) && (side_region == 2 || side_region== 3)) == 0)  
                    flag_new_region = 0;
                end
            end
            if flag_new_region == 1 %%%%%%%%%%55
               number_controls_less_equal(i) = number_controls_less_equal(i) + 1;  %%%%%%%%%%55
            end%%%%%%%%%%55
            
% % % %             if flag_new_region == 1 %%1
% % % %                 %set_new_regions = [set_new_regions; nodes{index_node,4}(j,1)]; %%1
% % % %                 set_new_regions = [set_new_regions; Regions(nodes{index_node,4}(j,1),:)]; %%1
% % % %             end %%1
        end
% % % %         set_controls_node = simplified_list_control_laws(set_new_regions,tol); %%1
% % % %         
% % % %         number_controls_less_equal(i) = size(set_controls_node,1);%%  %%1
    end
    
    %Greater Side
    for i = 1:size(set_new_index_ineq,1)
        set_test_index = [branch_index; set_new_index_ineq(i)];
        side_set_test_index = [nodes{index_node,8}; 2];
        set_new_regions = {}; %%%
        for j = 1:size(nodes{index_node,4},1)
            
            flag_new_region = 1;
            for k = 1:size(set_test_index,1)
                side_region = set_ineq{set_test_index(k,1),6}(nodes{index_node,4}(j,1),1);
                if (((side_set_test_index(k) == 1) && (side_region == 1 || side_region== 3)) == 0 && ((side_set_test_index(k) == 2) && (side_region == 2 || side_region== 3)) == 0)  
                    flag_new_region = 0;
                end
            end
            if flag_new_region == 1 %%%%%%%%%%55
               number_controls_greater_equal(i) = number_controls_greater_equal(i) + 1; %%%%%%%%%%55
            end %%%%%%%%%%55
% % % %             if flag_new_region == 1  %%1
% % % %                  %Regions_remaining = [Regions_remaining; Regions(nodes{index_node,4}(i,1),:)]; %%1
% % % %                  set_new_regions = [set_new_regions; Regions(nodes{index_node,4}(j,1),:)]; %%1
% % % %             end
        end
% % % %         set_controls_node = simplified_list_control_laws(set_new_regions,tol); %%1
% % % %         
% % % %         number_controls_greater_equal(i) = size(set_controls_node,1); %% %%1

    end
    
    
    
    
% % %         for j = 1:size(controls,1)
% % %             if sum(ismember([branch_index; set_new_index_ineq(i)], [controls{j,4};controls{j,6}])) == size([branch_index; set_new_index_ineq(i)],1)
% % %             %%%%if sum(ismember([branch_index; set_new_index_ineq(i)], controls{j,4})) == size([branch_index; set_new_index_ineq(i)],1)
% % %                 number_controls_less_equal(i) = number_controls_less_equal(i) + 1;
% % %                 
% % % %                 if number_controls_less_equal(i) >  num_max
% % % %                     num_max = number_controls_less_equal(i);
% % % %                     index_max = set_new_index_ineq(i);
% % % %                 end
% % %             end
% % %            if sum(ismember([branch_index; set_new_index_ineq(i)], [controls{j,4};controls{j,6}])) == size([branch_index; set_new_index_ineq(i)],1)
% % %            %%%%5if sum(ismember([branch_index; set_new_index_ineq(i)], controls{j,6})) == size([branch_index; set_new_index_ineq(i)],1)
% % %                 number_controls_greater_equal(i) = number_controls_greater_equal(i) + 1;
% % %                 
% % % %                  if number_controls_greater_equal(i) >  num_max
% % % %                     num_max = number_controls_greater_equal(i);
% % % %                     index_max = set_new_index_ineq(i);
% % % %                 end
% % %                 
% % %             end             
% % %         end
% % %     end
    
    %Create vector with max number of control laws and check the one with
    %less controls
    num_min = 0;
    index_min = 0;
% % % % % %     num_max = 0;
% % % % % %     index_max = 0;
    for i = 1:size(set_new_index_ineq,1)
        if number_controls_less_equal(i,1) > number_controls_greater_equal(i)
            number_max_control(i,1) = number_controls_less_equal(i);
        else
            number_max_control(i,1) = number_controls_greater_equal(i);
        end
        
        if ((number_max_control(i,1) < num_min && number_max_control(i,1) ~= 0) || (num_min == 0))%(index_min == 0))
            num_min = number_max_control(i,1);
            index_min = set_new_index_ineq(i);
        end
% % % %          if ((number_max_control(i) > num_max) || (i == 1))
% % % %             num_max = number_max_control(i);
% % % %             index_max = set_new_index_ineq(i);
% % % %         end
        
    end
    
    
    if index_min > 0
        index_ineq = index_min;
    else
        disp('deu ruimm')
        index_ineq = set_new_index_ineq(1);
    end
  
    
end

