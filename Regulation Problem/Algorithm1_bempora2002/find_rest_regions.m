function [CRest] = find_rest_regions(A, b, Nu, Nstate, out_region)
%[CRest] = find_rest_regions(A, b, Nu, Nstate, out_region)
%Find the rest regions from the polyehdral defined by Ax<=b inside an region.
%Inputs:
%       A, b - matrices that define the polyhedral Ax <= b
%
%       out_region - describes the limits of the regions, i.e. it is the 
%                    state constraints when creating the CR0
%
%Outputs:
%        CRest - a cell array with the N new rest regions, the first column
%                of the ith row is the Ai and the second column is the bi
%                from the ith region.
%
%Algoritm based on the paper "The explicit linear quadratic regulator for
%constrained systems" by A. Bemporad, M. Morari, V. Dua, and E. Pistikopoulos. 

Nx = 0;
    for i=1:(size(A,1))
        if (A(i,:) == [1 0])
            Nx= Nx+1; 
        elseif (A(i,:) == [0 1])
            Nx= Nx+1; 
        elseif (A(i,:) == [-1 0])
            Nx= Nx+1; 
        elseif (A(i,:) == [0 -1])  
            Nx= Nx+1;     
        end
    end

    Nr=Nx;
    for i = 1:(size(A,1)-Nr)
         A_Ri_rest = [];
         b_Ri_rest = [];
            if (size(A,1)-Nr)>1
                for j = 1:(i-1)
                    %CRest{i,1} = [CRest{i,1}; A(j,:)];
                    %CRest{i,2} = [CRest{i,2}; b(j,:)];
                    A_Ri_rest = [A_Ri_rest ; A(j,:)];
                    b_Ri_rest = [b_Ri_rest ; b(j,:)];
                end
            end
            i;
            %A(i,:)
            %CRest{i,1} = [CRest{i,1}; -A(i,:)];
            %CRest{i,2} = [CRest{i,2}; -b(i,:)];
            A_Ri_rest = [A_Ri_rest ; -A(i,:)];
            b_Ri_rest = [b_Ri_rest ; -b(i,:)];
            
            new_A =  A_Ri_rest;
            new_b =  b_Ri_rest;
            
            if isempty(out_region) == 0
                %new_A = CRest{i,1};
                %new_A = new_A(1:(size(new_A,1)-Nx),:);
  
                
                %new_b = CRest{i,2};
                %new_b = new_b(1:(size(new_b,1)-Nx),:);

                new_A = [new_A; out_region{1,1}];
                new_b = [new_b; out_region{1,2}];
                %CRest{i,1} = [new_A; A((size(A,1)-Nx+1):size(A,1),:); out_region{1,1}];
                %CRest{i,2} = [new_b; b((size(b,1)-Nx+1):size(b,1),:); out_region{1,2}];

            end
            [CRest{i,1}, CRest{i,2}] = remove_redundant_constraints(new_A, new_b, Nu, Nstate);
            
    end
    


end

