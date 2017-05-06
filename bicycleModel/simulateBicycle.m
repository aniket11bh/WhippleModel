% This file simulates the cycleycle motion for specific paths
% cycleycle model at constant velocity

close all
clear all
clc

% Load Bicycle parameters
addpath('params');
addpath('helperFunctions');
bicycleParam;

% Time parameters
dt = 0.01;
Tf = 10;    % simulation time = 10 seconds
tspan = 0:dt:Tf;
step = Tf/dt;

% Storage
Y = zeros(length(tspan), 1+10);

% Balancing mecha. Spring const. and damping const.
cycle.states = {'phi' 'delta' 'phi_dot' 'delta_dot'};
cycle.inputs = {'steerT'};
cycle.outputs = {'phi','delta'};

cycle.B2 = [0 0
                   0 0
                   bic.invM];

cycle.B = cycle.B2(:,2);

cycle.C = [1 0 0 0;
                 0 1 0 0];

cycle.D = [0];

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
    Yt = bicycleInputOutput( v, cycle.B2, cycle.C, cycle.D, u, Yt, ti, dt );

    Y(iterate, :) = Yt;
    bicycleAnimation(Yt);

    iterate = iterate + 1;
end

plotAllState(Y);
