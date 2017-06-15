close all
clear all
clc

% Load Bicycle parameters
addpath('params');
addpath('helperFunctions');
addpath('bicycleModel');
bicycleParam;
dcMotorParam;

% Time parameters
dt = 0.01;
Tf = 50;   % simulation time = 10 seconds
tspan = 0:dt:Tf;
step = Tf/dt;

% Get the trajectory
X = generateTrajectory(TrajectoryFlag);

% Storage
Y = [];

% Initial position of robot
X0 = [0 0 0 0]';    % initial conditions of the states
v = 1;  %[m/s]
x_i = 2;
y_i = 0.5;
psi_i = 0;

Yt = zeros(1,11);
Yt(2:5) = X0;
Yt(6:8) = [psi_i, x_i, y_i];
Yt(10) = v;
itr = 1;
inode = 1;

if strcmp(TrajectoryFlag, 'search')
    La = 1; % Look ahead distance
elseif strcmp(TrajectoryFlag, 'preplan')
    U = getRefinputsForInnerloop(X);

else
    fprintf('Not a trajectory planner type');
end

% Iterate over velocity %
for ti = 0:dt:Tf

    % Get current position of robot and velocity vector
    x = Yt(7:8)';
    V = Yt(10:11)';

    % Search based trajectory planning
    if strcmp(TrajectoryFlag, 'search')
        % Search for relevant segments %
        [inode, eps, Q1, Q2, Dap, L, Z] = propagateNode(X, x, inode);
        fprintf('Relevant segment : %d \n',inode);
        fprintf('\t x : %0.2f, y : %0.2f\n',x(1), x(2));
        fprintf('\t L : (%0.2f, %0.2f), eps : %0.2f, Dap : %0.2f, Z : %d\n',L(1), L(2), eps, Dap, Z);
        
        % Find goal point
        G = findGoal(inode, eps, Q1, Q2, Dap, L, X, x, La, Z);

        if Z == 0
            % eta : angle between G vector and current velocity vector
            eta = angle(V, G - x);
            k = 2*sin(eta)/La;

            % Steering angle req. in rad
            delta = atan(2*bic.w*sin(eta)/La);

             % Ref. steer angle, u
            u = [0;
                    delta];
            fprintf('\t eta : %.2f, delta_d : %.2f, steerA : %.2f, node : %d \n', eta*180/pi, delta*180/pi, Yt(3)*180/pi, inode);

            simulateTrajectoryControl(X, Yt, G, La);
        end

    % Preplanned trajectory
    elseif strcmp(TrajectoryFlag, 'preplan')
        u = [0
                U(2,inode) ];
        v = U(1,inode);
        simulateTrajectoryControl(X, Yt, 0, 0);
    else
        fprintf('Wrong trajectory flag \n');
    end

    Yt = bicycleInputOutput(v, u, Yt, ti, dt );
    
    Y = [Y; Yt];
    itr = itr + 1;

    if inode >= length(X)
        break
    end

end

plotAllState(X,Y);
