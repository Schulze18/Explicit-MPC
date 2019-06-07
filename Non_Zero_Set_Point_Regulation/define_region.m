function [A, b, type, origem] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol)
%[A, b] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol)
%
%Find the set of matrices inequalities that define the critical region.
%Inputs:
%       G, W and S - from the matrix inequality G*z <= W + S*x(t)
%
%       G_tio, W_tio and S_tio - rows of G, W and S corresponding to the active constraints
%
%       H - matrix from the cost function
%
%       tol - tolerance to consider something zero
%
%Outputs:
%       A, b - matrices that define the polyhedral Ax <= b
%
%       type - array with the type from each hyperplane A(i)x <= b(i)
%
%       origem - array of the constraints that generated the hyperplane A(i)x <= b(i)
%
%Algoritm based on the paper "An Algorithm for Multi-Parametric Quadratic Programming and 
% Explicit MPC Solutions" by P. Tondel, T. A. Johansen, and A. Johansen. 


    T = (inv(H)*G_tio'*inv(G_tio*inv(H)*G_tio'));
    
    %Equation 12
    A_1 = G*T*S_tio-S;
    b_1 = W - G*T*W_tio;
    
    %Equation 13
    A_2 = inv(G_tio*inv(H)*G_tio')*S_tio;
    b_2 = -inv(G_tio*inv(H)*G_tio')*W_tio;
    
    A = [];
    b = [];
    type = [];
    
    
    %Indicate that the constraints from Eq. 13 are not related to other index, this information is only used to make easier the code
    %Indica que as restrições geradas pela eq 13 nao estao relacionadas a nenhum outro indice, não é exatamente isso, mas facilita o algoritmo
    %origem = zeros(size(A_2,1),1);
    origem = [];
    
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
            type = [type ; 2];
            origem = [origem; i];
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
            type = [type; 1];   
            origem = [origem; i]; %Salva a origem das restrições ativas
        end
   
    end    
end

