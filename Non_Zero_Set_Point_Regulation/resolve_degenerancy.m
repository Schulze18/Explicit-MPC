function  [ G_tio, W_tio, S_tio, index, flag] = resolve_degenerancy(G, W, S, H, F, A, b, Nstate, Ncontrol, Nout, Ny, Nu, Lcand, tol)
%UNTITLED2 Summary of this function goes here 
%   Detailed explanation goes here
    %%%Reconstroi Região que originou a que esta sendo avaliada
    indices_old = [];
%     if length(Lcand) == 1
%         indices_old = [];
%         A = -S;
%         b = W;
%         origem = (1:size(G,1))';
%         type = ones(size(G,1),1);
%         [A, b, type, origem] = remove_redundant_constraints(A, b, type, origem, Nu, Nstate);
%     else
%         indices_old = Lcand(1:(end-1),:);
%         [ G_tio, W_tio, S_tio] = build_active_const(G, W, S, indices_old);
%         [A, b, type, origem] = define_region(G, W, S, G_tio, W_tio, S_tio, H, tol);
%         [A, b, type, origem] = remove_redundant_constraints(A, b, type, origem, Nu, Nstate);
%     end
    %%%
    %indices_old
    %Lcand
    %A = Lcand{1,2};
    %b = Lcand{1,3};
    index = [];
    G_tio = [];
    W_tio = [];
    S_tio = [];

    [xc , r, diagnostics] = chebychev_ball(A, b, G, W, S, H, F, Nstate, Ncontrol, Nout, Ny, Nu);

    for i = 1:size(xc,1)
        if isnan(xc(i))
            xc(i) = 0;
        end
    end
    
    if  (diagnostics.problem > 0)% || (sum(isnan(xc)) > 0) 
        flag = 1;
    else
        [z0, diagnostics] = optimal_z_mp_QP(G, W, S, H, F, xc, Nstate, Ncontrol, Nout, Ny, Nu);
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
%             options.solver='sedumi';
%             options.verbose = 0;
            
            options.solver='sdpt3';
            options.verbose = 0;
            options.cachesolvers = 1;
            
            options.sdpt3.maxit = 100;
            options.sdpt3.steptol = 1.0000e-05;
            options.sdpt3.gaptol = 5.000e-5;
            
            
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

