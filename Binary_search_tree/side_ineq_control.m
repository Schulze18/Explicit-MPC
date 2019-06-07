function [ ineq ] = side_ineq_control(ineq, controls)
%[ ineq ] = side_ineq_control(ineq, controls)
%
%Classify each control for each inequation
%Inputs:
%       ineq - set of inequations
%
%       control - set of control laws
%
%Outputs:
%       ineq - set of inequations with the information for each control law
%
    for i = 1:size(ineq,1)
        
        ineq{i,7} = zeros(size(controls,1),1);
        
        for j = 1:size(controls,1)
            for  n = 1:size(controls{j,3},1)              
%                 if ((i == 3) && (j == 3))
%                     disp('aqui')
%                 end
                if ineq{i,6}(controls{j,3}(n,1),1) == 3
                    ineq{i,7}(j,1) = 3;
                elseif (ineq{i,6}(controls{j,3}(n,1),1) == 1 && ineq{i,7}(j,1) == 2)
                    ineq{i,7}(j,1) = 3;
                elseif (ineq{i,6}(controls{j,3}(n,1),1) == 2 && ineq{i,7}(j,1) == 1)
                    ineq{i,7}(j,1) = 3;
                elseif (ineq{i,6}(controls{j,3}(n,1),1) == 1 && ineq{i,7}(j,1) == 0)
                    ineq{i,7}(j,1) = 1;
                elseif (ineq{i,6}(controls{j,3}(n,1),1) == 2 && ineq{i,7}(j,1) == 0)
                    ineq{i,7}(j,1) = 2;   
                end
            end
             
            for  n = 1:size(controls{j,5},1)
                if ineq{i,6}(controls{j,5}(n,1),1) == 3
                    ineq{i,7}(j,1) = 3;
                elseif (ineq{i,6}(controls{j,5}(n,1),1) == 1 && ineq{i,7}(j,1) == 2)
                    ineq{i,7}(j,1) = 3;
                elseif (ineq{i,6}(controls{j,5}(n,1),1) == 2 && ineq{i,7}(j,1) == 1)
                    ineq{i,7}(j,1) = 3;
                elseif (ineq{i,6}(controls{j,5}(n,1),1) == 1 && ineq{i,7}(j,1) == 0)
                    ineq{i,7}(j,1) = 1;
                elseif (ineq{i,6}(controls{j,5}(n,1),1) == 2 && ineq{i,7}(j,1) == 0)
                    ineq{i,7}(j,1) = 2;   
                end
             end
        end
    
    end
    

end

