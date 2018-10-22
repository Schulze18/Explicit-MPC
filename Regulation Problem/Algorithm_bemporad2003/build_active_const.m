function [G_tio, W_tio, S_tio] = build_active_const(G, W, S, index)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
    G_tio = [];
    S_tio = [];
    W_tio = [];
    if (length(index)>0)
        for i = 1:length(index)
            G_tio = [G_tio; G(index(i),:)];
            S_tio = [S_tio; S(index(i),:)];
            W_tio = [W_tio; W(index(i),:)];
        end
    end

end

