function [ all_possible_const] = generate_all_possible_constraints(G, W, S, H, tol)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    for i = 1:size(G,1)
        G_tio = G(i,:);
        S_tio = S(i,:);
        W_tio = W(i,:);
% %  ==       
%         G_tio = G;
%         W_tio = W;
%         S_tio = S;
%         G_tio(i,:) = [];
%         S_tio(i,:) = [];
%         W_tio(i,:) = [];
        
        [A, b, type] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol);

        all_possible_const{i,1} = A;
        all_possible_const{i,2} = b;
    end
end

