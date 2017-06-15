% Clear all %
clear all;
close all;
clc;


% Bicycle model

% Load Bicycle parameters
addpath('params');
addpath('bicycleModel');
addpath('helperFunctions');

bicycleParam;

% Record velocities at which system is uncontrollable
V = [];
eigenV = [];
iterate = 1;

% Iterate over velocity %
for v = 0:0.01:7
    
    A = getStateTransitionMatrix(v, bic.B, 3);
    B = bic.B; 
    C = bic.C;
    D = bic.D;
    
    sys_bic = ss(A, B, C, D);

    N_bar = dcgain(sys_bic);   

    sys_bic_norm  = ss(A, B/N_bar, C, D);
    
    eigenV = [eigenV eig(A)];
    
    stable = isstable(sys_bic_norm);
    
    fprintf('velocity : %.3f with result: %d\n', v, stable);
    
    % compute controllability
    if ( stable ~= 1)
        V = [V v];
        fprintf('\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\n');
        fprintf('Found unstable v : %.2f',v);
        fprintf('\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\n');
        pause;
    end
        
    iterate = iterate + 1;
end

plotEigenValues(eigenV,  0:0.01:7);