% This file simulates the bicycle motion for specific paths
% bicycle model at constant velocity

close all
clear all
clc

% Load Bicycle parameters
addpath('params');
bicycleParams;

invM = inv(M);
b31 = invM(1,1);
b42 = invM(2,2);

% Time parameters
dt = 0.01;
Tf = 10;    % simulation time = 10 seconds
tspan = 0:dt:Tf;         
step = Tf/dt;

u = pi/4;
X0 = [0 0 0 0]';    % initial conditions of the three states
v = 0.1;  %[m/s]

% title('Response to Non-Zero Initial Conditions')

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

A_bic = getStateTransitionMatrix(invM, v, B_bic_2);

sys_bic_cl  = ss( A_bic, B_bic, C_bic, D_bic, ...
                                'statename', bic_states,...
                                'inputname', bic_inputs,...
                                'outputname', bic_outputs);
N_bar = dcgain(sys_bic_cl);         

[t, X] = ode45( @(t,X) bicycleStateDot(t, X, A_bic, B_bic, u), tspan, X0);

figure(1)
plot(t, (180/pi)*X(:,1)/N_bar(1), t, (180/pi)*X(:,2)/N_bar(2));
title('Bicycle lean and steer');
xlabel('Time t (sec)');
ylabel('angle (degrees)');
legend('\phi','\delta');

% States as obtained from ode (in rad and rad/s respect.)
phi_dot = X(:,3);
delta_dot = X(:,4);
phi = X(:,1);
delta = X(:,2);

% yaw rate (rad/s) and yaw (rad)
psi_dot  = v*tan(X(:,2))/w; 
[t, psi] = ode45( @(t,psi) psi_dot( floor(t/dt) + 1) , tspan, 0);
    % convert yaw in to -180 to 180 range
    psi = atan2(sin(psi), cos(psi));

% Position calculation [m]
x_dot = v*cos(psi);
y_dot = v*sin(psi);

[t, x] = ode45( @(t,psi) x_dot( floor(t/dt) + 1) , tspan, 0);
[t, y] = ode45( @(t,psi) y_dot( floor(t/dt) + 1) , tspan, 0);

figure(2)
plot(t, (180/pi)*psi);
title('Yaw angle');
xlabel('Time t (sec)');
ylabel('angle (degrees)');
legend('Yaw angle of the bicycle');

figure(3)
plot(x,y);
title('Trajectory of the bicycle');
xlabel('x (m)');
ylabel('y (m)');
% legend('y_1','y_2')   