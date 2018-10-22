clear all
close all


x0 = [-1.8 0.4]';%[-0.28 -0.55]'

H = [1.079 0.076; 0.076 1.073];
F = [1.109 1.036; 1.573 1.517];
G = [1 0; 0 1; -1 0; 0 -1; 0.05 0; 0.05 0.05; -0.05 0; -0.05 -0.05];
W = [1 1 1 1 0.5 0.5 0.5 0.5]';
S = [1 0.9 -1 -0.9 0.1 0.1 -0.1 -0.1; 
     1.4 1.3 -1.4 -1.3 -0.9 -0.9 0.9 0.9]';

 index = [1 2 6];
%  G_tio = []
%  W_tio = [];
%  S_tio = [];
%  
%  for i = 1:length(index)
%     S_tio = [S_tio; S(i,:)];
%     W_tio = [W_tio; W(i,:)];
%     G_tio = [G_tio; G(i,:)];
%  end
 
for i = 1:1
    lambda = -inv(G(index(i),:)*inv(H)*G(index(i),:)')*(W(index(i),:) + S(index(i),:)*x0);
end
z  = inv(H)*G(index(i),:)'*lambda 
% z = z(1);

%%
lambda1 = sdpvar(1,1);
lambda7 = sdpvar(1,1);
x0 = [-0.28 -0.55]';
objetivo = lambda7;

LMI = [];
LMI = [LMI; lambda1 >= 0]
LMI = [LMI; lambda7 >= 0]

% LMI = [LMI; H*z + [G(1,:) ; G(7,:)]'*[lambda1; lambda7] == 0]
% for i = 1:size(index,1)
%       LMI = [LMI; H*z + G(index(i),:)*lambda1 == 0]
% end
% LMI = [LMI; H*z + G(index(i),:)'*lambda1 == 0];
% LMI = [LMI; H*z + G(index(i),:)'*lambda7 == 0];
LMI = [LMI; H*z + [G(1,:) ; G(7,:)]'*[lambda1; lambda7] == 0];
%LMI = [LMI; [G(1,:) ; G(7,:)]*z <= [W(1,:) ; W(7,:)] + [S(1,:) ; S(7,:)]*x0];

options=sdpsettings;
options.solver='sedumi';
solvesdp(LMI,-objetivo,options);
double(lambda1)
double(lambda7)

%%
q = size(G,1);
Nr = 0;
for k = 0:(2^q-1)
    Nr = Nr + q^k*factorial(k)
end


