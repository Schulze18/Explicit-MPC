function [ status_check ] = check_if_verified(possible_new_set, Lopt)
%[ status_check ] = check_if_verified(possible_new_set, Lopt)
%
%Check if a combination of constraints was already verified
%Inputs:
%       possible_new_set - candidate set of constraints
%
%       Lopt - list of already verified constraints
%
%Outputs:
%       status_check - true if it was already verified
    possible_new_set = sort(possible_new_set);
    
    status_check = false;
    
    if isequal(Lopt,possible_new_set)
        status_check = true;
    else
        for i = 1:size(Lopt,1)
            if isequal(Lopt{i,1},possible_new_set)
                status_check  = true;
                break
            elseif isempty(Lopt{i,1}) && isempty(possible_new_set)
                status_check  = true;
                break
            end
        end
    end
end

