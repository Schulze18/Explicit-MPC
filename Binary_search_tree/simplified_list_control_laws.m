function [controls] = simplified_list_control_laws(Regions, tol)
%[controls] = list_control_laws(Regions, set_ineq, tol)
%List all the control laws in the Regions without comparing do inequations
%Inputs:
%       Regions - cell array with all the regions from the explicit
%       controller
%
%       tol - tolerance to consider something zero     
%
%Outputs:
%       controls - list of the control laws
%   
%   
    %%%%controls = Kx Kc regions_pos ineq_pos regions_neg ineq_neg 
    % controls = Kx Kc regions_within
    controls = {};
    for i = 1:size(Regions,1)
        
        flag_new_control = 1;
        
        for j = 1:size(controls,1)
            if verify_equal_matrices(controls{j,1},Regions{i,3},tol) && verify_equal_matrices(controls{j,2},Regions{i,4},tol)
                flag_new_control = 0;
            end
        end
        
        if flag_new_control
              controls{end+1,1} = Regions{i,3};
              controls{end,2} = Regions{i,4};      
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

