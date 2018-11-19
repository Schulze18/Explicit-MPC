function [rep_vetor] = index_opt_repition(Lopt)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

    rep_vetor = zeros(size(Lopt,1),1);
    for i = 1:size(Lopt,1)
        for j = 1:size(Lopt,1)
            if (j~=i && isequal(Lopt{j},Lopt{i}))
                rep_vetor(i) = rep_vetor(i) + 1;
                j
                i
            end
        end
    
    end

end

