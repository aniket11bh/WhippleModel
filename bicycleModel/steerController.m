% This file tunes steer angle response only at particular v

% Bicycle model
clear all
close all
clc

% Load Bicycle parameters
addpath('params');
bicycleParams;

invM = inv(M);
b31 = invM(1,1);

v = 0;
C = v*C1;
K = g*K0 + v*v*K2;

s1 = -( invM(2,1)*K(2,1) + invM(2,2)*K(2,2));
s2 = -( invM(2,1)*C(2,1) + invM(2,2)*C(2,2));

A_steer = [ 0  1;
                      s1 s2 ];

B_steer = [0
                      invM(2,2)];
 
C_steer = [1 0];

D_steer = [0];

% Balancing mecha. Spring const. and damping const.
bic_states = {'delta','delta_dot'};
bic_inputs = {'steerT'};
bic_outputs = {'delta'};

%%%%%%%%%% Pole placement %%%%%%%%%%%%%%
% Design req. : 
%               Mp < 0.1 rad
%               tr < 1 sec
%           zeta = 0.6, wn = 10

p_steer = [-3+1i , -3-1i];

K_steer = place(A_steer, B_steer, p_steer);

sys_steer_cl  = ss((A_steer - B_steer*K_steer), B_steer, C_steer, D_steer, ...
                                    'statename', bic_states,...
                                    'inputname', bic_inputs,...
                                    'outputname', bic_outputs);
              
N_bar = dcgain(sys_steer_cl);              
t = 0:0.01:4;

% Initial condition : 0 rad, 0 rad/s
    % x0 = [pi/4 0]; 

x0 = [0 0]; 

u = pi/4*ones(size(t));

[y, t, x] = lsim(sys_steer_cl/N_bar, u, t, x0);
plot(t, y*180/pi, t, u*180/pi, ':' );
title('Open loop response');
xlabel('Time (sec)');
ylabel('Lean angle (deg)');

% Same as leanController impulse creates a huge 6 rad. impulse.. 
    % impulse((180/pi)*sys_steer_cl/N_bar);