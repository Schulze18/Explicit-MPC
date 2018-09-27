function [ status_check ] = check_if_verified(possible_new_set, Lopt)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
    possible_new_set = sort(possible_new_set);
    
    status_check = false;
    for i = 1:size(Lopt,1)
        if isequal(Lopt{i,1},possible_new_set)
            status_check  = true;
            break
        end
    end  
end

