function [ reg_struct ] = regions_cell_to_struct(Regions)
%[ reg_struct ] = regions_cell_to_struct(Regions)
%
%Convert cell array with Regions to struct data type
%Inputs:
%       Regions - a cell array with the partitioned regions, each row is
%       composed by four columns: Ai bi Kx Kc
%
%Outputs:
%       reg_struct - struct array with four fields: : A b Kx Kc
%
max_size = 1;
for i = 1:size(Regions,1)
    if size(Regions{i,1},1) > max_size
       max_size = size(Regions{i,1},1);
    end
end
max_size
for i = 1:size(Regions,1)
    reg_struct(i).A = [Regions{i,1}; zeros(max_size-(size(Regions{i,1},1)),size(Regions{i,1},2))];
    reg_struct(i).b = [Regions{i,2}; zeros(max_size-(size(Regions{i,1},1)),1)];
    reg_struct(i).Kx = Regions{i,3};
    reg_struct(i).Kc = Regions{i,4};
end

