% Clear all %
clear all;
close all;
clc;

% Load Bicycle parameters %
bicycleParams;

invM = inv(M);
b31 = invM(1,1);
b42 = invM(2,2);

%{
% s = tf('s');
% G = b31/(s*s - invM(1,2)*C(2,1)*s - invM(1,1)*g*K0(1,1) - invM(1,2)*g*K0(2,1));

% sisotool(G);

% with Kp = 2.58e+04, Kd = 1.43e+03, Tf = 0.000317
%}

% Record velocities at which system is uncontrollable
V = [];
iterate = 1;

% Iterate over velocity %
for v = 0:0.001:10
   
    B_bic_2 = [0 0
                        0 0
                        invM];

    B_bic = B_bic_2(:,2);
 
    C_bic = [1 0 0 0;
                     0 1 0 0];

    D_bic = [0];
    
    A_bic = getStateTransitionMatrix(invM, v, B_bic_2);
    
    sys_bic_cl  = ss(( A_bic), B_bic, C_bic, D_bic);
    
    stable = isstable(sys_bic_cl);
    
    fprintf('velocity : %.3f with result: %d\n', v, stable);
    
    % compute controllability
    if ( stable ~= 1)
        V = [V v];
        fprintf('\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\n');
        fprintf('Found unstable v : %.2f',v);
        fprintf('\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\n');
        pause;
    end
    
%     N_bar = dcgain(sys_bic_cl);         
%     step(sys_bic_cl(1)/N_bar(1), sys_bic_cl(2)/N_bar(2) );
    
    iterate = iterate + 1;
end