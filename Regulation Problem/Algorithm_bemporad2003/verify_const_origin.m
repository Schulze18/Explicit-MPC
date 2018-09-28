function [ origem ] = verify_const_origin(A, b, G, W, S, z0, all_possible_const)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    INEQ_A = -S;
    INEQ_b = W - G*z0;
    %origem = {};
    origem = zeros(size(A,1),1);
%     
%     for i = 1:size(A,1)
%         for j = 1:size(G,1)
% %             if A(i,1) == 0 
% %                 if (A(i,2)/INEQ_A(j,2) == b(i,1)/INEQ_b(j,1))
% %                     origem(i) = j;
% %                     break
% %                 end
% %             else
% %             if (A(i,1)/INEQ_A(j,1) == b(i,1)/INEQ_b(j,1))
% %                  origem(i) = j;
% %                  break
% %             end
%               if ((A(i,1)/INEQ_A(j,1) == A(i,2)/INEQ_A(j,2)) && (INEQ_A(j,1)*INEQ_b(j))*(A(i,1)*b(i)) >=0)
%                 origem(i) = j;
%                 %break
%               end
% 
%         end
%     end
%     end

%     for i = 1:size(A,1)
%         for j = 1:size(INEQ_b,1)
%             combination_index = combnk(1:size(INEQ_b,1),j);
%             for k = 1:size(combination_index ,1)
%                 test_comb = [A(i,:) b(i)];
%                 for n = 1:j
%                     test_comb = [test_comb; INEQ_A(combination_index(k,n),:)  INEQ_b(combination_index(k,n),1)];
%                 end
%                 if rank(test_comb) == j
%                     combination_index(k,:)
%                     origem{i} = combination_index(k,:);
%                 end
%             end
%             
%         end
%     end
    for i = 1:size(A,1)
        for j = 1:size(G,1)
            A_test = all_possible_const{j,1};
            b_test = all_possible_const{j,2};
            for k = 1:size(A_test,1)
                if ((A(i,1) == A_test(k,1)) && (A(i,2) == A_test(k,2)) && (A(i,2) == A_test(k,2)) && (b(i) == b_test(k))) 
                    if (origem(i) ~= 0)
                        disp('nope');
                    end
                    origem(i) = k;
                end
            end
        end
    end



end

