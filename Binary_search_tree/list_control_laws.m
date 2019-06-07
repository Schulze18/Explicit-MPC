function [controls] = list_control_laws(Regions, set_ineq, tol)
%[controls] = list_control_laws(Regions, set_ineq, tol)
%List all the control laws in the explicit MPC
%Inputs:
%       Regions - cell array with all the regions from the explicit
%       controller
%
%       set_ineq - set of inequations
%
%       tol - tolerance to consider something zero     
%
%Outputs:
%       controls - list of all the control laws
%   
    %%%%controls = Kx Kc regions_pos ineq_pos regions_neg ineq_neg 
    % controls = Kx Kc regions_within
    controls = {};
    for i = 1:size(Regions,1)
        
        flag_new_control = 1;
        
        for j = 1:size(controls,1)
            if verify_equal_matrices(controls{j,1},Regions{i,3},tol) && verify_equal_matrices(controls{j,2},Regions{i,4},tol)
                flag_new_control = 0;
                
% % % % % % % % % % % % % % % % % % % %                 if ismember(i,controls{j,3}) == 0
% % % % % % % % % % % % % % % % % % % %                     controls{j,3}(end+1,1) = i;
% % % % % % % % % % % % % % % % % % % %                 end
% % % % % % % % % % % % % % % % % % % %                 
                ineq_number = 0;
                for n = 1:size(set_ineq,1)
                    flag_in_region = verify_ineq_in_region(Regions(i,:), set_ineq(n,:));
                    
                    %Store if it is in <= or >= definition 
                    if  flag_in_region == 1
                        controls{j,3}(end+1,1) = i; %Region
                        controls{j,4}(end+1,1) = n; %Ineq
                        ineq_number = ineq_number + 1;
                    elseif flag_in_region == 2
                        %if controls{j,5}
                        controls{j,5}(end+1,1) = i;
                        controls{j,6}(end+1,1) = n;
                        ineq_number = ineq_number + 1;
                    end  
                end
                if ineq_number ~= size(Regions{i,1},1)
                    disp('ruim')
                end
            end
        end
        
        if flag_new_control
              controls{end+1,1} = Regions{i,3};
              controls{end,2} = Regions{i,4};
              controls{end,3} = i;
              controls{end,3} = [];
              controls{end,4} = [];
              controls{end,5} = [];
              controls{end,6} = [];
              
              for n = 1:size(set_ineq,1)
                  flag_in_region = verify_ineq_in_region(Regions(i,:), set_ineq(n,:));
                  %Store if it is in <= or >= definition
                  if  flag_in_region == 1
                       controls{end,3}(end+1,1) = i;
                       controls{end,4}(end+1,1) = n;
                      controls{end,3} = i;
                      controls{end,4} = n;
                      controls{end,5} = [];
                      controls{end,6} = [];
                  elseif  flag_in_region == 2
                       controls{end,5}(end+1,1) = i;
                       controls{end,6}(end+1,1) = n;
                      controls{end,3} = [];
                      controls{end,4} = [];
                      controls{end,5} = i;
                      controls{end,6} = n;
                  end
              end          
        end
    end
    
%     %Organize the data
%     for i = 1:size(controls,1)
%         controls{i,3} = sort(controls{i,3});
%         controls{i,4} = sort(controls{i,4});
%         controls{i,5} = sort(controls{i,5});
%         controls{i,6} = sort(controls{i,6});
%     end
    
    
end

