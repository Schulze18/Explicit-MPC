function [ flag_equal ] = verify_equal_matrices(M1,M2,tol)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    flag_equal = 1;
    for i = 1:size(M1,1)
        for j = 1:size(M1,2)
            if abs(M1(i,j) - M2(i,j)) > tol
                flag_equal = 0;
            end
        end
    end 
end