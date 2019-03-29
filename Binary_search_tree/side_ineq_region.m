function [result, diagnostics1, diagnostics2] = side_ineq_region(ineq, Region, options)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    %result = 0; 
    tol = 1e-5;

    %Test if ineq is in Region definition
    result = verify_ineq_in_region(Region, ineq);
% % %     A_region = Region{1,1};
% % %     b_region = Region{1,2};
% % %     
% % %     A_ineq = ineq{1,1};
% % %     b_ineq = ineq{1,2};
% % %     
% % %     for i = 1:size(A_region,1)
% % %         test_matrix = [A_region(i,:) b_region(i,:); A_ineq b_ineq]; 
% % %         rank_test = rank(test_matrix);
% % %                 
% % %         sign_test = sum(test_matrix(1,:))/sum(test_matrix(2,:));
% % %     
% % %         if rank_test == 1 && sign_test > 0
% % %             result = 1;
% % %             break
% % %         elseif rank_test == 1 && sign_test < 0
% % %             result = 2;
% % %             break
% % %         end
% % %    
% % %     end
  
    %Only if it is not from the Region Definition
    if result == 0
    
        A_ineq = ineq{1,1};
        b_ineq = ineq{1,2};
       
        %Check <= sign
        x = sdpvar(size(A_ineq,2),1,'full');

        objective = A_ineq*x - b_ineq;
% % % % %         options = sdpsettings;
% % % % % 
% % % % %         options.solver='sedumi';
% % % % % %          options.solver='sdpt3';
% % % % %         options.verbose = 0;
% % % % %         options.cachesolvers = 1;
% % % % % 
% % % % % %         options.sdpt3.maxit = 20;
% % % % % %         options.sdpt3.steptol = 1.0000e-08;
% % % % % %         options.sdpt3.gaptol = 5.000e-8;
% % % % %         
% % % % %         options.sedumi.eps = 5.0000e-04;
% % % % %         options.sedumi.maxiter = 20;


        A_region = Region{1,1};
        b_region = Region{1,2};

        LMI1 = [];
        for i = 1:size(A_region,1)
            LMI1 = [LMI1; A_region(i,:)*x <= b_region(i,:) - tol];
        end

        diagnostics1 = optimize(LMI1,objective,options);

        if double(objective) <= -tol || isequal( diagnostics1.info,'Unbounded objective function (SeDuMi-1.3)')
            result = result + 1; 
        end

        %Check >= sign
        x2 = sdpvar(size(A_ineq,2),1,'full');
        objective2 = -(A_ineq*x2 - b_ineq);
        LMI2 = [];
        for i = 1:size(A_region,1)
            LMI2 = [LMI2; A_region(i,:)*x2 <= b_region(i,:)-tol];
        end

        diagnostics2 = optimize(LMI2,objective2,options);

        if double(objective2) <= -tol || isequal( diagnostics2.info,'Unbounded objective function (SeDuMi-1.3)')
            result = result + 2; 
        end
        
        %%% Debug
%         if (isequal( diagnostics1.info,'Unbounded objective function (SeDuMi-1.3)') && (double(objective) > -tol))
%             disp('unbounded 1')
%         end
%         
%         if (isequal( diagnostics2.info,'Unbounded objective function (SeDuMi-1.3)') && (double(objective2) > -tol))
%             disp('unbounded 2')
%         end
        if result == 0
            disp('deu ruim')
        end
    
        
        
    end
     
end

