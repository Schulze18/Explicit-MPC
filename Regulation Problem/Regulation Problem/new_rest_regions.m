function [ CRest ] = new_rest_regions(A,b,out_region, old_out_region)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    A_rest = A;
    b_rest = b;
    A_out = old_out_region{:,1};
    CRest{1,1}=[];
    CRest{1,2}=[];
    Nx = 0;
%     for i=1:(size(A,1))
%         if (A(i,:) == [1 0])
%             Nx= Nx+1; 
%         elseif (A(i,:) == [0 1])
%             Nx= Nx+1; 
%         elseif (A(i,:) == [-1 0])
%             Nx= Nx+1; 
%         elseif (A(i,:) == [0 -1])  
%             Nx= Nx+1;     
%         end
%     end
%     
%     Nout = 0;
%     for i=1:(size(A_out,1))
%         if (A_out(i,:) == [1 0])
%              
%         elseif (A_out(i,:) == [0 1])
%           
%         elseif (A_out(i,:) == [-1 0])
%             
%         elseif (A_out(i,:) == [0 -1])  
%                
%         else
%             Nout = Nout+1;
%         end
%     end
    
%     Nx
%     Nout
%     Nr = Nout + Nx
%     size(A,1)-Nr
%     A
%     b
%     Nr = Nx
    
    if (size(A,1)-Nr)>1
        for i= 2:(size(A,1)-Nr)
            CRest{i,1} = A(1,:);
            CRest{i,2} = b(1,:);
        end
    end
    
    for i = 1:(size(A,1)-Nr)
        if (size(A,1)-Nr)>1
            for j = 2:(i-1)
                CRest{i,1} = [CRest{i,1}; A(j,:)];
                CRest{i,2} = [CRest{i,2}; b(j,:)];
            end
        end
        CRest{i,1} = [CRest{i,1}; -A(i,:)];
        CRest{i,2} = [CRest{i,2}; -b(i,:)];

        if isempty(out_region) == 0
            new_A = CRest{i,1};
            %new_A = new_A(1:(size(new_A,1)-Nx),:);
            new_b = CRest{i,2};
            %new_b = new_b(1:(size(new_b,1)-Nx),:);
      
            CRest{i,1} = [new_A; out_region{1,1}];
            CRest{i,2} = [new_b; out_region{1,2}];
            %CRest{i,1} = [new_A; A((size(A,1)-Nx+1):size(A,1),:); out_region{1,1}];
            %CRest{i,2} = [new_b; b((size(b,1)-Nx+1):size(b,1),:); out_region{1,2}];
            
        end
    end
end

