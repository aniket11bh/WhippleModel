close all
clear all
clc

% Load Bicycle parameters
addpath('params');
addpath('helperFunctions');
addpath('bicycleModel');
bicycleParams;

% Time parameters
dt = 0.01;
Tf = 50;    % simulation time = 10 seconds
tspan = 0:dt:Tf;         
step = Tf/dt;

% Get the trajectory
X = generateTrajectory();

% Storage 
Y = zeros(length(tspan), 1+10);

% Bicycle model params
B_bic_2 = [0 0
                    0 0
                    invM];

B_bic = B_bic_2(:,2);
            
C_bic = [1 0 0 0;
                 0 1 0 0];

D_bic = [0];

% Initial position of robot
X0 = [0 0 0 0]';    % initial conditions of the states
v = 1;  %[m/s]
x_i = 0;
y_i = 0;
psi_i = 0;

Yt = zeros(1,11);
Yt(2:5) = X0;
Yt(6:8) = [psi_i, x_i, y_i];
Yt(10) = v;

inode = 1;
itr = 1;
La = 1; % Look ahead distance

% Iterate over velocity %
for ti = 0:dt:Tf
      
    % Get current position of robot and velocity vector
    x = Yt(7:8)';
    V = Yt(10:11)';
       
    G = [];

    while isempty(G)
        
        % Search for relevant segments %
        [inode, eps, Q1, Q2, Dap, L, Z] = propagateNode(X, x, inode);
        
        fprintf('Relevant node : %d \n',inode);
        
        % Find goal point
        G = findGoal(inode, eps, Q1, Q2, Dap, L, X, La, Z);
        
        if isempty(G)
            inode = inode + 1;
        end
                
    end
   
    % eta : angle between G vector and current velocity vector
    eta = angle(G - x, V);
    k = 2*sin(eta)/La;
     
    % Steering angle req. in rad
    delta = -atan(2*w*sin(eta)/La);
    
    % Ref. steer angle, u
    u = delta;
    fprintf('\t delta_d : %f, next node : %d \n',delta, inode); 
    
    Yt = bicycleInputOutput(v, B_bic_2, C_bic, D_bic, u, Yt, ti, dt );
    simulateTrajectoryControl(X, Yt, G, La);
    Y(itr, :) = Yt;
    itr = itr + 1;
    
    if inode >= length(X)
        break
    end
end

plotAllState(Y);