function [A, b] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol)
%[A, b] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol)
%
%Find the set of matrices inequalities that define the critical region.
%Inputs:
%       G, W and S - from the matrix inequality G*z <= W + S*x(t)
%
%       G_tio, W_tio and S_tio - rows of G, W and S corresponding to the active constraints
%
%Outputs:
%       A, b - matrices that define the polyhedral Ax <= b
%
%Algoritm based on the paper "The explicit linear quadratic regulator for
%constrained systems" by A. Bemporad, M. Morari, V. Dua, and E. Pistikopoulos. 


    T = (inv(H)*G_tio'*inv(G_tio*inv(H)*G_tio'));
    A_1 = G*T*S_tio-S;
    b_1 = W - G*T*W_tio;
    A_2 = inv(G_tio*inv(H)*G_tio')*S_tio;
    b_2 = -inv(G_tio*inv(H)*G_tio')*W_tio;
    
    A = [];
    b = [];
    
    for i = 1:size(A_2,1)
        flag = 0;
        
        for j = 1:size(A_2,2) 
            if(A_2(i,j) > tol || A_2(i,j) < -tol )
                flag = 1;
            end   
        end
        
        if flag == 1
            A = [A; A_2(i,:)];
            b = [b; b_2(i,:)];
        end
   
    end
    
    for i = 1:size(A_1,1)
        flag = 0;
        
        for j = 1:size(A_1,2) 
            if(A_1(i,j) > tol || A_1(i,j) < -tol )
                flag = 1;
            end   
        end
        
        if flag == 1
            A = [A; A_1(i,:)];
            b = [b; b_1(i,:)];
        end
   
    end    
end

