function [ result ] = side_ineq_region_intersec(ineq, Region, cheb_center, options)
%[result, diagnostics1, diagnostics2] = side_ineq_region(ineq, Region, options)
%
%Find if the "Region" is on the right or in the left of the "ineq" with the
%intersection test
%Inputs:
%       ineq - inequation definition
%
%       Region - region to be tested
%
%       cheb_center - Chebyshev center from the region
%
%       options - solver options for yalmip 
%
%Outputs:
%       result - indicate the side: 0 - no answer, 1 - left, 2 - right,
%       3 - both sides
%
    tol = 1e-6;
    %Test if ineq is in Region definition
    result = verify_ineq_in_region(Region, ineq);

    if result == 0 
        A_ineq = ineq{1,1};
        b_ineq = ineq{1,2};
        A_region = Region{1,1};
        b_region = Region{1,2};
%         
%         poly_test = Polyhedron('A', Region{1,1}, 'b', Region{1,2});
%         hyper = Polyhedron('Ae', A_ineq, 'be', b_ineq);
%         flag_intersect = poly_test.intersect(hyper).isFullDim()
%         %         if doesIntersect(poly_test, hyper, 'fully')

        x = sdpvar(size(A_ineq,2),1,'full');
        LMI = [];
        for i = 1:size(A_region,1)
            LMI = [LMI; A_region(i,:)*x <= b_region(i,:) - tol];
        end
        LMI = [LMI; A_ineq*x == b_ineq];
        
        diagnostics = optimize(LMI,[],options);
        
        %if strcmp(diagnostics.info,'Successfully solved (SeDuMi-1.3)')
        if  diagnostics.problem == 0
%             flag_intersect = 1;
            test_lmi = check(LMI);
            flag_lmi = 1;
            for i = 1:(size(test_lmi,1))
                if test_lmi(i) <= -tol
                    flag_lmi = 0;
                end
            end

            if flag_lmi == 1
                flag_intersect = 1;
            else
                flag_intersect = 0;
            end
            
        else
            flag_intersect = 0;
        end
            
        if flag_intersect == 1
           result = 3;
        else
            if (A_ineq*cheb_center <= b_ineq)
                result = 1;
            else
                result = 2;
            end
        end
        yalmip('clear');
    end
    
end

