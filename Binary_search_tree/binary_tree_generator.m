clear all
clc
tol = 1e-6;

load('regions_10_2_u_du.mat');
%Step 1 - Algorithm 2 - Computation all the I(j+) and I(j-)
ineq_pos = verifiy_total_ineq(Regions);
ineq_neg = cellfun(@(x) x*(-1),ineq_pos,'un',0);


%%
ineq_test = which_region_ineq(ineq_pos,Regions,tol);

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
%%
tic
clear nodes
last_index_ineq = 0;
index_ineq = 1;
unex_node = 1;
nodes{1,1} = ineq_pos{1,1};         %aj from dj
nodes{1,2} = ineq_pos{1,2};         %bj from dj
nodes{1,3} = [];
nodes{1,4} = (1:size(Regions,1))';     %regions for the node
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
    
%     %Define inequation to test

%     index_ineq = rem(index_node,size(ineq_pos,1));
%     if index_ineq == 0
%         index_ineq = size(ineq_pos,1);
%     end
    
    flag_new_ineq = 0;
    for i = 1:size(nodes{index_node,4},1)
        if flag_new_ineq == 1
                break
        end
        for j = 1:size(ineq_pos,1)
            if flag_new_ineq == 1
                break
            end
            %Verify inequation "not used" that define a Region 
            for k = 1:size(ineq_pos{j,3},1)
                if (ineq_pos{j,3}(k,1) == nodes{index_node,4}(i,1)) && (ismember(j,nodes{index_node,3}) == 0)%j ~= last_index_ineq
                    last_index_ineq = index_ineq;
                    index_ineq = j;
                    flag_new_ineq = 1;
                    break
                end
            end
        end
    end
    
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
    
    %Save last node
    nodes{nodes{index_node,5},7} =  index_node;
    nodes{nodes{index_node,6},7} =  index_node;
    
    %Save inequations from the branch
    nodes{nodes{index_node,5},3} = [nodes{index_node,3}];
    nodes{nodes{index_node,6},3} = [nodes{index_node,3}];
    
    for i = 1:size(nodes{index_node,4},1)
        if ineq_pos{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 1 || ineq_pos{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 3
            nodes{nodes{index_node,5},4} = [nodes{nodes{index_node,5},4}; nodes{index_node,4}(i,1)];
        end
        if ineq_pos{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 2 || ineq_pos{nodes{index_node,3}(end,1),6}(nodes{index_node,4}(i,1),1) == 3
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
