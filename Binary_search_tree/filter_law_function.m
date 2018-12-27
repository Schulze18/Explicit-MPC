function [ Regions_out] = filter_law_function(Regions, tol)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    Regions_out = Regions;
    for n = 1:size(Regions_out,1)
        for i = 1:size(Regions_out{n,3},1)
            for j = 1:size(Regions_out{n,3},2)
                if Regions_out{n,3}(i,j) < tol && Regions_out{n,3}(i,j) > - tol
                    Regions_out{n,3}(i,j) = 0;
                end
            end
            if Regions_out{n,4}(i,1) < tol && Regions_out{n,4}(i,1) > - tol
                Regions_out{n,4}(i,1) = 0;
            end
             
        end
    end
    
end

