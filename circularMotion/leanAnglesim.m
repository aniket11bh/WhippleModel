clear all
close all
clc

% Bicycle model

% Load Bicycle parameters
addpath('params');
addpath('helperFunctions');
addpath('bicycleModel');

bicycleParam;

% Bicycle velocity
v = 1.5;



% 
tspan = 0:0.1:20;

[t, y] = ode45(@(t, y) vdp(t, y, v), tspan, [0;0] );

figure
plot(t, y(:,1))
figure
plot(t, y(:,2))

function dydt = vdp(t, y, v)
    global bic g 
    C = v*bic.C1;
    K = g*bic.K0 + v*v*bic.K2;
    S1 = -bic.invM*K;
    S2 = -bic.invM*C;
    dydt = zeros(2,1);
    phi = y(1);
    phi_dot = y(2);
    dydt(1) = phi_dot;
    dydt(2) = (S1(1,1) - S1(1,2)*S1(2,1)/S1(2,2))*phi + (S2(1,1) - S1(1,2)*S2(2,1)/S1(2,2))*phi_dot;
end