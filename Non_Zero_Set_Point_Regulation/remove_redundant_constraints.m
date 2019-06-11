function [A_new, b_new, type_new, origem_new] = remove_redundant_constraints(A, b, type_new, origem_new, Nu, Nstate, options)
%[A_new, b_new] = remove_redundant_constraints(A, b, type_new, origem_new, Nu, Nstate, options)
%
%Remove the redundant constraints of a polyehdral defined by Ax<=b.
%Inputs:
%       A, b - matrices that may contain redundant constraints
%
%       Nu - control horizon
%
%       Nstate - number of states
%
%       type_new, origem_new - type of the original index that generated Ax<=b
%
%       origem_new -  original constraints that generated the hyperplane
%
%       options - solver options for yalmip 
%
%Outputs:
%       A_new, b_new - matrices without redundant constraints
%
%       type_new, origem_new - type and contraints from the reduced set of
%       contraints
%
%Algoritm based on the Lecture Notes "Polyhedral Computation - Spring 2014" from K.
%Fukuda.   

index = [];
      
%     options = sdpsettings;
%     options.solver = 'sdpt3';
%     options.verbose = 0;
%     options.cachesolvers = 1;
    x = sdpvar(Nstate,1,'full');
    for i=1:size(A,1)
        %x = sdpvar(Nstate,1,'full');
        LMI = [];
        for j=1:size(A,1)
            if j ~= i
                LMI = [LMI; A(j,:)*x <= b(j)];
            end
        end
        LMI = [LMI, A(i,:)*x <= (b(i)+1)];
        objetivo = A(i,:)*x;
        
% % %         options = sdpsettings;
% % %         options.solver = 'sdpt3';
% % % % %         options.solver = 'linprog';
% % % % % %         options.solver = 'sedumi';
% % %         options.verbose = 0;
% % %         options.cachesolvers = 1;
% % % % %%Sedumi
% % % % %         options.sedumi.cg.stagtol = 5.000e-4;
% % % % %         options.sedumi.eps = 1.0000e-04;
% % % % %         options.sedumi.maxiter = 15;
% % % %  
% % % % %%SDPT3
%         options.sdpt3.maxit = 30;
%         options.sdpt3.steptol = 1.0000e-5;
%         options.sdpt3.gaptol = 5.000e-5;
% % 
% % % % %%Linprog
% % %         options.linprog.TolCon = 1e-8;
% % %         options.linprog.TolFun = 1e-8;
% % 
        optimize(LMI,-objetivo,options);
        
        if(double(objetivo) <= b(i))
            index = [index, i];
        end
        
        
        
        %%%%%%%%%%%%%%
%%linprog
% %     f = -double(A(i,:)');
% %     A_ineq = double(A);
% %     b_ineq = double(b);
% %     b_ineq(i) = double(b_ineq(i)+1);
% %     options = optimoptions('linprog','Algorithm','dual-simplex');
% %     [x,fval,flag,output] =linprog(f,A_ineq,b_ineq,[],[],[],[],[],options);
% %     x =  double(x);
% %     if(output.constrviolation>0)
% %         rank(A_ineq)
% %         size(A_ineq)
% %         disp('conts violation');
% %     end
% % %     if (output.cgiterations>0)
% % %         disp('talvez,ruim');
% % %     end
% %     if(double(x)*A(i,:) <= b(i))
% %             index = [index, i];
% %     end    
        
        %%%%%%%%%%%%%%%%
        
%         
%         A_ineq = [];
%         B_ineq = [];
%         for j=1:size(A,1)
%             if j ~= i
%                  A_ineq = [A_ineq; A(j,:)];
%                  B_ineq = [B_ineq; b(j)];
%             end
%         end
%        A_ineq = [A_ineq; A(i,:)];
%        B_ineq = [B_ineq; (b(i)+1)];
       
%        H_quad = zeros(Nstate);
%        F_quad =  -A(i,:);
%        lb = zeros(Nstate,1);
%        opt = optimoptions('quadprog');
%        opt.MaxIterations = 20;
%        opt.Display = 'none';
%        opt.OptimalityTolerance = 1.0000e-08;
%        opt.StepTolerance = 5.000e-8;
%        
%        x = quadprog(H_quad, F_quad, A_ineq, B_ineq);%, [],[],[],[],[], opt);
        
% %       F_lin = -(A(i,:)');
% %       lb = -Inf*ones(Nstate,1);
% %       ub = Inf*ones(Nstate,1);
% %       opt = optimoptions('linprog','Display' ,'none');
% %        opt.Algorithm = 'interior-point';
% %        x = linprog(F_lin, A_ineq, B_ineq,[],[],lb,ub,[],opt)
% %     
% %        if(double(x)*A(i,:) <= b(i))
% %             index = [index, i];
% %        end
              
    end
%     A_new = [];
%     b_new = [];
%     for i = 1:size(A,1)
%         if i
%         A_new(i,:) = A(i,:);
%         b_new(i,:) = b(i,:);
%     
%     end
    
    A_new = A;
    b_new = b;
    index;
    for k = 1:size(index,2)
        A_new(index(k)-k+1,:) = [];
        b_new(index(k)-k+1,:) = [];
        type_new(index(k)-k+1,:) = [];
        origem_new(index(k)-k+1,:) = [];
    end

end

