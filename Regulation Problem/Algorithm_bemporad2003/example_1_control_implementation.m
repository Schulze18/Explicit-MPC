%System from Example 1 Bemporad 2002
A = [0.7326 -0.0861;
     0.1722 0.9909];
B = [0.0609; 0.0064];
C = [0 1.4142];

x = zeros(2,50);
y = zeros(50,1);
x(:,1) = [1.2 -1.4]'; 
for i = 1:50
   
    for j = 1:size(Regions,1)
        A_CRi = Regions{j,1};
        b_CRi = Regions{j,2};
        flag = 0;
        for k = 1:size(A_CRi,1)
            if(A_CRi(k,:)*x(:,i) > b_CRi(k))
                flag = 1;
            end
        end
        if flag == 0
            index = j;
        end
    end
    u(i) = Regions{index,3}*x(:,i) + Regions{index,4};
%     A*x(:,i)
%     B*u
    x(:,i+1) = A*x(:,i)+B*u(i)+0.02*[rand(1) ; rand(1) ];
    y(i) = C*x(:,i); 

end

% close(10)
% close(11)
figure(10)
stairs(x(1,:))
hold on
stairs(x(2,:),'r')
legend('x1','x2')

figure(11)
stairs(u)
legend('u')
