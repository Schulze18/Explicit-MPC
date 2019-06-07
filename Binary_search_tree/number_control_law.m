function [control_law, vector_number] = number_control_law(ineq, Regions)
%[control_law, vector_number] = number_control_law(ineq, Regions)
%Get the remaining of control laws in a set of Regions
%Inputs:
%       Regions - cell array with a set of the regions from the explicit
%       controller
%
%       ineq - set of inequations  
%
%Outputs:
%       control_law - cell array with the remaining control laws
%
%       vector_number - array with the number of control laws for each
%       inequation
    
    control_law = cell(size(ineq,1),1);
    vector_number = zeros(size(ineq,1),1);
    
    %Check each set of inequation
    for i = 1:size(ineq,1)
       %i
       A_set_ineq = ineq{i,1};
       b_set_ineq = ineq{i,2};
       
       %Check for each region
       number_rank = 0;
       for j = 1:size(Regions,1)
            A_region = Regions{j,1};
            b_region = Regions{j,2};
            
            flag = 0;
            number_in = 0;
            for n = 1:size(A_set_ineq,1)
                %Check if inequation defines an Regions

                for k = 1:size(A_region,1)
                    %Check linear dependecy between the rows
                    test_matrix = [A_region(k,:) b_region(k,:); A_set_ineq(n,:) b_set_ineq(n,:)]; 
                    rank_test = rank(test_matrix);
                    
                    if rank_test == 1
                        number_rank = number_rank +1; 
                    end
                    
                    %Check linear dependecy and if both are j+ or j-
                    if ((rank_test == 1) && (sum(test_matrix(1,:))/sum(test_matrix(2,:)) >= 0)) == 1
                        %Count number of inequations from the set that are
                        %in the region description
                        number_in = number_in + 1;
                        %flag = 0;
                        %break
                    end
                end
            end
             
            %Check if all the elements are in the region description
            if number_in == size(A_set_ineq,1)
                flag = 1;
%                 i
%                 j
            end
%             flag = (ismember(A_set_ineq, A_region,'rows') && ismember(b_set_ineq, b_region,'rows')); 
%             flag
            if flag == 1
                vector_number(i,1) = vector_number(i,1) + 1; 
                if isempty(control_law{i}) == 0
                    j
                    %Check for repetition in the control laws
%                     for 
%                     
%                     if
                    %if (ismember(Regions{j,3},control_law{i}{1,1},'rows') && ismember(Regions{j,4},control_law{i}{1,2},'rows')) == 0   
                        %control_law{i}{1,1}(end+1,:) = Regions{j,3};
                        %control_law{i}{1,2}(end+1,:) = Regions{j,4};
                        control_law{i}{end+1,1} = Regions{j,3};
                        control_law{i}{end,2} = Regions{j,4};
                        control_law{i}{end,3} = j;
                        control_law{i}{end,4} = number_rank;
                        %end
                else
                    control_law{i} = {Regions{j,3} Regions{j,4} j number_rank};
                end
            end   
       end
       control_law{i}{end,4} = number_rank;
       
    end
    
end

