function [Regions] = partition_rest_regions(Rest_regions, G, W, S, H, F, tol, Nu, Nstate, number_partition)
%[Regions] = partition_rest_regions(Rest_regions, G, W, S, H, F, tol, Nu, Nstate, number_partition)
%
%Partition the rest regions of the CR0.
%Inputs:
%       Rest_regions - a cell array with the rest regions, the first column
%                      of the ith row is the Ai and the second column is the
%                      bi from the ith region.
%                     
%       G, W, S, H and F - from the cost functions: 
%                          Vz(x) = 0.5*z'*H*z 
%                                  through z 
%                                  subject to G*z <= W + S*x(t)
%
%       tol - acceptable tolerance in the equation G_tio(i)*z0 = W + S*x0
%
%       Nu - control horizon 
%
%       number_partition - the number of times that the region was partitioned 
%
%Outputs:
%       Regions - a cell array with the partitioned regions, each row is
%       composed by four columns: Ai bi Kx Kc
%
%Algoritm based on the paper "The explicit linear quadratic regulator for
%constrained systems" by A. Bemporad, M. Morari, V. Dua, and E. Pistikopoulos. 

    number_partition
    Regions = {};
    for i = 1:size(Rest_regions,1)
        A = Rest_regions{i,1};
        b = Rest_regions{i,2};
        [ xc , r, diagnostics] = chebychev_ball( A, b, G, W, S, H, F, Nu, Nstate);  %Find epsilon and x0
        
        if (r <= 0.1 || isnan(r) || isnan(xc(1)) || (diagnostics.problem==1))
             
        else
            [ z0, diagnostics ] = optimal_z_mp_QP( G, W, S, H, F, xc, Nu);  % Find z0
            %z0 = fcnKKT(H, F, G, E, W, xc)
            
            G_tio = [];
            %if isempty(z0) == 0
            if diagnostics.problem == 0
                [G_tio W_tio S_tio] = verify_active_constraints(G, W, S, xc, z0, tol); %Find G_tio, W_tio e S_tio
            end
            %Define U(x)
            if isempty(G_tio) == 1
                %Region that is similar to a LQR
                Kx = (-inv(H)*F');
                Regions = [Regions; {Rest_regions{i,:}} Kx(1,:) 0];
                    
             elseif (rank(G_tio) < size(G_tio,1)) %|| rank(G_tio) < size(G_tio,2))
%                 disp('degeneracao')
%                 G_tio
                %Regions = [Regions; {Rest_regions{i,:}} 20];
            else
                
                %rank_degen = rank(G_tio);
                %if rank(W_tio) > rank_degen
                 %  rank_degen = rank(W_tio);
                %end
                %if rank(S_tio) > rank_degen
                 %  rank_degen = rank(S_tio); 
                %end
                
%                 if rank_degen < Nu
%                     rank_degen
%                     disp('degeneracao')
%                     G_tio = G(rank_degen,:)
%                     W_tio = W(rank_degen,:);
%                     S_tio = S(rank_degen,:);
%                 end
                
                [A, b] = define_region( G, W, S, G_tio, W_tio, S_tio, H, tol); %Define polyhedron
                [Kx, Ku] = define_control(G, W, S, G_tio, W_tio, S_tio, H, F);
                [A, b] = remove_redundant_constraints(A, b, Nu, Nstate);
                CR = {A b Kx Ku};
                Regions = [Regions; CR];
                
                %plotregion(-A, -b)
                
                %Rest_CR = new_rest_regions(A,b,{Rest_regions{i,:}}, old_out_region);
                Rest_CR = find_rest_regions(A,b,Nu, Nstate,{Rest_regions{i,:}});
              
                new_regions = partition_rest_regions(Rest_CR, G, W, S, H, F, tol, Nu, Nstate, number_partition+1);
                Regions = [Regions; new_regions];
                
                %Rest_CR{1,3} = 30;
                %Rest_CR{1,4} = 30;
                %Regions = [Regions; Rest_CR];
            end
        end
    end
end

