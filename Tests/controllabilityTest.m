clear all;
close all;
clc;

% Bicycle model %

% Load Bicycle parameters
bicycleParams;
invM = inv(M);

% Record velocities at which system is uncontrollable
V = [];
iterate = 1;

% Iterate over velocity %
for v = 0:0.001:10
    C = v*C1;
    K = g*K0 + v*v*K2;
    
    S1 = -invM*K;
    S2 = -invM*C;
    
    A = [ 0 0 1 0 ;
          0 0 0 1 ;
          S1  S2 ];

    B = [0 0
         0 0
        invM];
 
    C = [1 0 0 0;
         0 1 0 0];

    D = [0 0;
         0 0];

    rank_cb = rank(ctrb(A, B));
    
    fprintf('velocity : %.3f with rank : %d\n', v, rank_cb);
    
    % compute controllability
    if ( rank_cb ~= 4)
        V = [V v];
        fprintf('\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\n');
        fprintf('Found uncontrollable v : %.2f',v);
        fprintf('\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\n');
        pause;
    end
    
    iterate = iterate + 1;
end