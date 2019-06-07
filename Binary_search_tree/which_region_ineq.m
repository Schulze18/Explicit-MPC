function [ ineq_out ] = which_region_ineq(ineq, Regions,tol)
%[ ineq_out ] = which_region_ineq(ineq, Regions,tol)
%
%Verify the inequations present in the Regions definition
%Inputs:
%       ineq - set of inequations
%
%       Regions - cell array with the regions from the Explicit MPC
%
%       tol - tolerance to consider something zero
%
%Outputs:
%       ineq_out - set of inequation with regions and controls law
%
    ineq_out = ineq;
    for i = 1:size(ineq,1)
        ineq_out{i,3} = [];
        ineq_out{i,4} = {};
        %ineq_out{i,5} = [];
        %Verify each region
        for j = 1:size(Regions,1)
            A_region = Regions{j,1};
            b_region = Regions{j,2};
            %Verify each inequation of the region
            for k = 1:size(A_region,1)
                test_matrix = [A_region(k,:) b_region(k,:); ineq{i,1} ineq{i,2}]; 
                rank_test = rank(test_matrix);
                
                sign_test = sum(test_matrix(1,:))/sum(test_matrix(2,:));
                
                if rank_test == 1 && sign_test > 0
                    ineq_out{i,3} = [ineq_out{i,3}; j]; 
                    
                    %Add new control law
                    if isempty(ineq_out{i,4})
                        ineq_out{i,4} = {Regions{j,3} Regions{j,4}};
                    else
                        %Verify if it is a new control law
                        flag_new_control = 1;
                        for n = 1:size(ineq_out{i,4},1)
% %                             if isequal(ineq_out{i,4}{n,1},Regions{j,3}) && isequal(ineq_out{i,4}{n,2},Regions{j,4})
% %                                 flag_new_control = 0;
% %                             end
                              if verify_equal_matrices(ineq_out{i,4}{n,1},Regions{j,3},tol) && verify_equal_matrices(ineq_out{i,4}{n,2},Regions{j,4},tol)
                                    flag_new_control = 0;
                              end
                        end
                        if flag_new_control
                            ineq_out{i,4}{end+1,1} = Regions{j,3};
                            ineq_out{i,4}{end,2} = Regions{j,4};
                        end
                    end
               end
            end
        
        
        end
    
    end



end

