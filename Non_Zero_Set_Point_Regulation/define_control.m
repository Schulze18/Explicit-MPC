function [Kx, Kc] = define_control(G, W, S, G_tio, W_tio, S_tio, H, F, Ncontrol)
%[Kx, Kc] = define_control(G, W, S, G_tio, W_tio, S_tio, H, F)
%
%Find the control law associated to the critical region.
%Inputs:
%       G, W, S, H and F - from the cost function: 
%                          Vz(x) = 0.5*z'*H*z 
%                                 through z 
%                                 subject to G*z <= W + S*x(t)
%
%       G_tio, W_tio and S_tio - rows of G, W and S corresponding to the active constraints
%
%       Ncontrol - number of control actions
%
%Outputs:
%       Kx and Kc - coefficients from the control feedback law u(x) = Kx*x(t) + Kc
%
%Algoritm based on the paper "The explicit linear quadratic regulator for
%constrained systems" by A. Bemporad, M. Morari, V. Dua, and E. Pistikopoulos. 

    T = inv(H)*G_tio'*inv(G_tio*inv(H)*G_tio');
    Kx = (T*S_tio-inv(H)*F');
    Kc = T*W_tio;
    Kx = Kx(1:Ncontrol,:);
    Kc = Kc(1:Ncontrol,:);
end

