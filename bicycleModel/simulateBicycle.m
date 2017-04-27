% This file simulates the bicycle motion for specific paths
% bicycle model at constant velocity

close all
clear all
clc

% Load Bicycle parameters
addpath('params');
addpath('helperFunctions');
bicycleParams;

% Time parameters
dt = 0.01;
Tf = 10;    % simulation time = 10 seconds
tspan = 0:dt:Tf;         
step = Tf/dt;

% Storage
Y = zeros(length(tspan), 1+10);

% Balancing mecha. Spring const. and damping const.
bic_states = {'phi' 'delta' 'phi_dot' 'delta_dot'};
bic_inputs = {'steerT'};
bic_outputs = {'phi','delta'};

B_bic_2 = [0 0
                    0 0
                    invM];

B_bic = B_bic_2(:,2);
            
C_bic = [1 0 0 0;
                 0 1 0 0];

D_bic = [0];

iterate = 1;

% Initial conditions
X0 = [0 0 0 0]';    % initial conditions of the states
v = 1;  %[m/s]
Yt = zeros(1,11);
Yt(2:5) = X0;

% Iterate over velocity %
for ti = 0:dt:Tf
    
    % Ref. steer angle, u
    u = pi/4;
    Yt = bicycleInputOutput( v, B_bic_2, C_bic, D_bic, u, Yt, ti, dt );
    
    Y(iterate, :) = Yt;
    simulateTrajectory(Yt);
    
    iterate = iterate + 1;
end

plotAllState(Y);