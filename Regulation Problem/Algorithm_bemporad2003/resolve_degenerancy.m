function  [ G_tio, W_tio, S_tio, index, flag] = resolve_degenerancy(G, W, S, H, F, A, b, Nu, Nstate, Lcand)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    index = [];
    G_tio = [];
    W_tio = [];
    S_tio = [];


    [xc , r, diagnostics] = chebychev_ball(A, b, G, W, S, H, F, Nu, Nstate);
    
    if  diagnostics.problem > 0
        flag = 1;
    else
        [z0, diagnostics] = optimal_z_mp_QP(G, W, S, H, F, xc, Nu);
        if diagnostics.problem > 0
            flag = 1;
        else
            G_test = [];
            for i = 1: length(Lcand)
                G_test = [G_test ; G(Lcand(i),:)];
            end
            
            
            %Equacao 18, 19 e 20
            lambda = sdpvar(length(Lcand),1,'full');
            
            objective = -lambda(end);
            LMI = [];
            LMI = [LMI; H*z0 + G_test'*lambda == 0];
            LMI = [LMI; lambda >= 0];
            
            options=sdpsettings;
            options.solver='sedumi';
            options.verbose = 0;
            diagnostics = optimize(LMI,objective,options);
            
            
            if diagnostics.problem > 0
                flag = 1;
            else
                lambda = double(lambda);
                flag = 0;
                index = [];
                for i = 1:length(Lcand)
                    if lambda(i) == Inf
                        flag = 1;
                    elseif lambda(i) > 0
                        index = [index; Lcand(i)];
                        G_tio = [G_tio; G(Lcand(i),:)];
                        S_tio = [S_tio; S(Lcand(i),:)];
                        W_tio = [W_tio; W(Lcand(i),:)];
                    end
                end
            end
            
        end
    end     
end

