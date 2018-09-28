Nc = 2;
Nu = 2;
Ny = 2;

A=[-0.1044   -0.1214;
   -3.4152    4.6425;
    3.4152   -4.6425;
    1.0000         0];
b = [-0.0353;
    1.3658;
    2.6342;
    1.5000];
Umax = 2;
Umin = -2;
G = [ 1  0;
     -1  0;
      0  1;
      0  -1];
W = [Umax; -Umin; Umax; -Umin];

E = zeros(Nc*Nu,Nu);
z0 = zeros(2,1);
%Adição da restrição no estado 1.5 < x1,x2 < 1.5
W = [W; 1.5; 1.5; 1.5; 1.5];
E = [E; -1 0; 0 -1; 1 0; 0 1];
G = [G; zeros(4,2)];

S = E + G*inv(H)*F';
test_A = [];
test_B = [];

INEQ_A = -S;
INEQ_b = W - G*z0;
figure
hold on
for i = 1:size(A,1)
        %for j = 1:size(INEQ_b,1)
         for j = 3:size(INEQ_b,1)
            combination_index = combnk((1:size(INEQ_b,1))',j);
            for k = 1:size(combination_index ,1)
                %test_comb = A(i,:);
                %combination_index(k,:)
                test_A = [];
                test_b = [];
                for n = 1:j
                    %test_comb = [test_comb; INEQ_A(combination_index(k,n),:)];
                    test_A = [test_A ; INEQ_A(combination_index(k,n),:)];
                    test_B = [test_B ; INEQ_b(combination_index(k,n),:)];
                end
                test_A
                test_B
                      plotregion(-test_A,-test_B)
                %test_comb
            end        
        end
end


for n = 1:j
    test_comb = [test_com; INEQ_A(combination_index(n))];
end