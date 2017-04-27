function [A] = getStateTransitionMatrix( invM, v, B )
%GET_AB_MATRICES returns A and B matrices
%   input: v speed of the bicycle

global C1;
global K0;
global  K2;
global g;

C = v*C1;
K = g*K0 + v*v*K2;
S1 = -invM*K;
S2 = -invM*C;

A    = [ 0 0 1 0 ;
             0 0 0 1 ;
             S1  S2 ];
         
K_lean = zeros(1,4);
K_lean(1) = 7843.20023833066;
K_lean(3) = 424.726009043642;

K_steer = zeros(1,4);
K_steer(2) = 9.46120848195432;
K_steer(4) = 1.387655359381;

A = A - B(:,1)*K_lean - B(:,2)*K_steer;

end

