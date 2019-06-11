function [ u, index, cont_reg ] = test_struct(X, ref, index_old)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

    hue = load('struct_regions.mat');
    struct_regions = hue.reg_struct;
    cont_reg = 0;
    index = 0;
    for j = 1:size(struct_regions,2)
            A_CRi =  struct_regions(j).A;
            b_CRi =  struct_regions(j).b;
            flag = 0;
            for k = 1:size(A_CRi,1)
                if(A_CRi(k,:)*[X; ref] > b_CRi(k))
                    flag = 1;
                end
            end
            if flag == 0
                cont_reg = cont_reg + 1; 
                index = j;
            end
    end
    if index == 0
        index = index_old;
    end
    u_calc = double(struct_regions(index).Kx*[X; ref] + struct_regions(index).Kc)
    u = u_calc(1:4,1);
end

