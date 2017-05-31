function [A] = getStateTransitionMatrix( v, B, type )
%GET_AB_MATRICES returns A and B matrices
%   input: v speed of the bicycle
%   type: "0, 1, 2, 3" : A :: A - B*K_l :: A-B*K*s :: A-B*Ks-B*Kl"

global bic g;

C = v*bic.C1;
K = g*bic.K0 + v*v*bic.K2;
S1 = -bic.invM*K;
S2 = -bic.invM*C;

A    = [ 0 0 1 0 ;
             0 0 0 1 ;
             S1  S2 ];

if type == 0
    A = A;
    
elseif type  == 1
    K_lean = zeros(1,4);
    K_lean(1) = 7843.20023833066;
    K_lean(3) = 424.726009043642;
    
    A = A - B(:,1)*K_lean;
    
elseif type == 2
        
    K_steer = zeros(1,4);
    K_steer(2) = 9.46120848195432;
    K_steer(4) = 1.387655359381;
    
    A = A - B(:,2)*K_steer;

elseif type == 3
    K_lean = zeros(1,4);
    K_lean(1) = 7843.20023833066;
    K_lean(3) = 424.726009043642;

    K_steer = zeros(1,4);
    K_steer(2) = 9.46120848195432;
    K_steer(4) = 1.387655359381;

    A = A - B(:,1)*K_lean - B(:,2)*K_steer;
else
    fprintf('Wrong input');
end


end

