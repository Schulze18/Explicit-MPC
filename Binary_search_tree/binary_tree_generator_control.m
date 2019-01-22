clear all
clc
tol = 1e-6;

load('regions_10_2_u.mat');
%Step 1 - Algorithm 2 - Computation all the I(j+) and I(j-)
ineq_pos = verifiy_total_ineq(Regions);
ineq_neg = cellfun(@(x) x*(-1),ineq_pos,'un',0);


%%
%ineq_test = which_region_ineq(ineq_pos,Regions,tol);

%%
tic
for i = 1:size(ineq_pos,1)
    i
    for j = 1:size(Regions,1)
        result = side_ineq_region(ineq_pos(i,:),Regions(j,:));
        ineq_pos{i,6}(j,1) = result;
    end
end
toc

%% List all the control laws
controls = list_control_laws(Regions, ineq_pos, tol);
% % % % 
% % % % old_controls = controls;
% % % % index_remove = [];
% % % % for i = 1:size(controls,1)
% % % %     count_remove_less = 0;
% % % %     count_remove_greater = 0;
% % % %     
% % % %     if isempty(controls{i,4}) == 0
% % % %         
% % % %         for j = 1:size(old_controls{i,4},1)
% % % % %             count_remove_greater = 0;
% % % %             flag_remove = 0;
% % % %             if isempty(controls{i,6}) == 0
% % % %                 for n = 1:size(old_controls{i,6},1)
% % % %                     %             if ismember(old_controls{i,4}(j,1),old_controls{i,6})
% % % %                     if old_controls{i,4}(j,1) == old_controls{i,6}(n,1)
% % % %                         flag_remove = 1;
% % % %                         i
% % % %                         j
% % % %                         n
% % % %                         controls{i,4}(j -  count_remove_less,:) = [];
% % % %                         controls{i,3}(j -  count_remove_less, :) = [];
% % % %                         controls{i,5}(n -  count_remove_greater,:) = [];
% % % %                         controls{i,6}(n -  count_remove_greater,:) = [];
% % % %                         %count_remove_less = count_remove_less + 1;
% % % %                         count_remove_greater = count_remove_greater + 1;
% % % %                     end
% % % %                 end
% % % %             end
% % % %             if flag_remove
% % % %                 count_remove_less = count_remove_less + 1;
% % % %             end
% % % %         end
% % % %     end
% % % % end
% for i = 1:size(controls,1)
%     controls{i,3} = []; controls{i,4} = []; controls{i,5} = []; controls{i,6} = [];
%     for j = 1:size(old_controls{i,4},1)
%         if ismember(old_controls{i,4}(j,1),index_remove) == 0
%             controls{i,3}(end+1,1) =  old_controls{i,3}(j,1);
%             controls{i,4}(end+1,1) =  old_controls{i,4}(j,1);
%         end
%     end
% end

% % controls = old_controls;

%%
ineq_pos = side_ineq_control(ineq_pos, controls);

%%
tic
clear nodes
last_index_ineq = 0;
index_ineq = 1;
unex_node = 1;
nodes{1,1} = ineq_pos{1,1};         %aj from dj
nodes{1,2} = ineq_pos{1,2};         %bj from dj
nodes{1,3} = [];
%nodes{1,4} = (1:size(Regions,1))';     %regions for the node
nodes{1,4} = (1:size(controls,1))';     %controls for the node
nodes{1,5} = 2;                     %<= node
nodes{1,6} = 3;                     %>= node
nodes{1,7} = [];                    %parent node

%%
% nodes(nodes{1,5},:) = cell(1,7);%[]; nodes{nodes{1,5},2} = []; nodes{nodes{1,5},3} = []; nodes{nodes{1,5},4} = []; nodes{nodes{1,5},5} = []; nodes{nodes{1,5},6} = []; nodes{nodes{1,5},7} = 1;
%     nodes(nodes{1,6},:) = cell(1,7);%[]; nodes{nodes{1,6},2} = []; nodes{nodes{1,6},3} = []; nodes{nodes{1,6},4} = []; nodes{nodes{1,6},5} = []; nodes{nodes{1,6},6} = []; nodes{nodes{1,6},7} = 1;
%     for i = 1:size(nodes{1,4},1)
%         if ineq_pos{nodes{1,3}(end,1),6}(i,1) == 1 || ineq_pos{nodes{1,3}(end,1),6}(i,1) == 3
%             nodes{nodes{1,5},4} = [nodes{nodes{1,5},4}; i];
%         end
%         if ineq_pos{nodes{1,3}(end,1),6}(i,1) == 2 || ineq_pos{nodes{1,3}(end,1),6}(i,1) == 3
%             nodes{nodes{1,6},4} = [nodes{nodes{1,6},4}; i];
%         end
%     end
% 
%     if size(nodes{nodes{1,5},4},1) > 1
%         unex_node = [unex_node; nodes{1,5}]; 
%     end
%     if size(nodes{nodes{1,6},4},1) > 1
%         unex_node = [unex_node; nodes{1,6}]; 
%     end

%%

it_max = 100;
it = 0;
while isempty(unex_node) == 0 && it < it_max 
    it = it + 1;
    
    index_node = unex_node(end,1);
    
    %Define inequation to the node
    
    index_ineq = define_inequation_node_control(nodes, index_node, ineq_pos, nodes{index_node,3}, controls);
    %index_ineq = define_inequation_node(nodes, index_node, ineq_pos, nodes{index_node,3}, controls);
    %index_ineq = define_inequation_node_old(nodes, index_node, ineq_pos);
    
    
%     flag_new_ineq = 0;
%     for i = 1:size(nodes{index_node,4},1)
%         if flag_new_ineq == 1
%                 break
%         end
%         for j = 1:size(ineq_pos,1)
%             if flag_new_ineq == 1
%                 break
%             end
%             %Verify inequation "not used" that define a Region 
%             for k = 1:size(ineq_pos{j,3},1)
%                 if (ineq_pos{j,3}(k,1) == nodes{index_node,4}(i,1)) && (ismember(j,nodes{index_node,3}) == 0)%j ~= last_index_ineq
%                     last_index_ineq = index_ineq;
%                     index_ineq = j;
%                     flag_new_ineq = 1;
%                     break
%                 end
%             end
%         end
%     end
    
    
    nodes{index_node,1} = ineq_pos{index_ineq,1};                   %aj from dj
    nodes{index_node,2} = ineq_pos{index_ineq,2};                   %bj from dj
    nodes{index_node,3} = [nodes{index_node,3}; index_ineq];        %inequation index
    
    nodes(end+1,:) = cell(1,7);
    nodes(end+1,:) = cell(1,7);
   
    nodes{index_node,5} = size(nodes,1) - 1;             %<= node   
    nodes{index_node,6} = size(nodes,1);                 %>= node
    
    %Initialize node
    nodes(nodes{index_node,5},:) = cell(1,7);
    nodes(nodes{index_node,6},:) = cell(1,7);
    
    %Save parent node
    nodes{nodes{index_node,5},7} =  index_node;
    nodes{nodes{index_node,6},7} =  index_node;
    
    %Save inequations from the branch
    nodes{nodes{index_node,5},3} = [nodes{index_node,3}];
    nodes{nodes{index_node,6},3} = [nodes{index_node,3}];
    
    %Verify if the region in this node are in <= or >= side
    for i = 1:size(nodes{index_node,4},1)
        %if ineq_pos{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 1 || ineq_pos{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 3
        if ineq_pos{nodes{index_node,3}(end,1),7}(nodes{index_node,4}(i,1),1) == 1 || ineq_pos{nodes{index_node,3}(end,1),7}(nodes{index_node,4}(i,1),1) == 3
            nodes{nodes{index_node,5},4} = [nodes{nodes{index_node,5},4}; nodes{index_node,4}(i,1)];
        end
        %if ineq_pos{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 2 || ineq_pos{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 3
        if ineq_pos{nodes{index_node,3}(end,1),7}(nodes{index_node,4}(i,1),1) == 2 || ineq_pos{nodes{index_node,3}(end,1),7}(nodes{index_node,4}(i,1),1) == 3
            nodes{nodes{index_node,6},4} = [nodes{nodes{index_node,6},4}; nodes{index_node,4}(i,1)];
        end
    end

    unex_node(end,:) = [];
    if size(nodes{nodes{index_node,5},4},1) > 1
        unex_node = [unex_node; nodes{index_node,5}]; 
    end
    if size(nodes{nodes{index_node,6},4},1) > 1
        unex_node = [unex_node; nodes{index_node,6}]; 
    end

end
toc

% % %% 
% % vetor_rep_controls = zeros(size(Regions,1),3);
% % for i = 1:size(Regions,1)
% %     Kx = Regions(i,3);
% %     Kc = Regions(i,4);
% %     
% %     for j = 1:size(Regions,1)
% %         Kx_test = Regions(j,3);
% %         Kc_test = Regions(j,4);
% %         
% %         if (isequal(Kx_test,Kx) && isequal(Kc,Kc_test))
% %             vetor_rep_controls(i,1) = vetor_rep_controls(i,1) + 1;
% %             
% %             if (vetor_rep_controls(i,2) == 0 && i~=j)
% %                 vetor_rep_controls(i,2) = j;
% %             elseif (vetor_rep_controls(i,2) > 0 && i~=j)
% %                 vetor_rep_controls(i,3) = j;
% %             end
% %             
% %         end
% %     end
% % end
% % 
% % %%
% % for i = 1:size(control_law,1)
% %     if control_law{i}{4} > 1
% %         i
% %     end
% % end
% % 
% % 
% % %%
% % vetor_rep_ineq = zeros(size(ineq_pos,1),1);
% % for i = 1:size(ineq_pos,1)
% %     for j = 1:size(Regions,1);
% %         A_region = Regions{j,1};
% %         b_region = Regions{j,2};
% %         %test_matrix = [A_regions b_regions; ineq_pos{i,1} ineq_pos{i,2}];
% %         %if rank(test_matrix) == size(Regions,1)
% % %         if ismember([ineq_pos{i,1} ineq_pos{i,2}],[A_regions b_regions]) 
% % %             vetor_rep_ineq(i) = vetor_rep_ineq(i) + 1;
% % %         end
% %         
% %         for k = 1:size(A_region,1)
% %             %Check linear dependecy between the rows
% %             test_matrix = [A_region(k,:) b_region(k,:); ineq_pos{i,1} ineq_pos{i,2}];
% %             rank_test = rank(test_matrix);
% %             
% %             if rank_test == 1
% %                 vetor_rep_ineq(i) = vetor_rep_ineq(i) + 1;
% %             end
% %             
% %             %Check linear dependecy and if both are j+ or j-
% % %             if ((rank_test == 1) && (sum(test_matrix(1,:))/sum(test_matrix(2,:)) >= 0)) == 1
% % %                 %Count number of inequations from the set that are
% % %                 %in the region description
% % %                 vetor_rep_ineq(i) = vetor_rep_ineq(i) + 1;
% % %                 %flag = 0;
% % %                 %break
% % %             end
% %         end
% %         
% %         
% %         
% %     end
% % end
% % 
