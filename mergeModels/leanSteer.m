% This file basically combines leanController file with the complete
% bicycle model

clear all
close all
clc

% Bicycle model

% Load Bicycle parameters
addpath('params');
bicycleParams;

invM = inv(M);
b31 = invM(1,1);
b42 = invM(2,2);

v = 0;
C = v*C1;
K = g*K0 + v*v*K2;

S1 = -invM*K;
S2 = -invM*C;


A_bic = [ 0 0 1 0 ;
                  0 0 0 1 ;
                  S1  S2 ];

B_bic_2 = [0 0
                    0 0
                    invM];

B_bic = B_bic_2(:,2);
                
C_bic = [1 0 0 0;
                0 1 0 0];

D_bic = [0];

% Balancing mecha. Spring const. and damping const.
bic_states = {'phi' 'delta' 'phi_dot' 'delta_dot'};
bic_inputs = {'steerT'};
bic_outputs = {'phi','delta'};

%%%%%%%%%% Pole placement %%%%%%%%%%%%%%
% Design req. : 
%               Mp < 0.1 rad
%               tr < 1 sec
%           zeta = 0.6, wn = 10
% p_bic = [-6+8i , -6-8i, -100, -50];

K_lean = zeros(1,4);
K_lean(1) = 7843.20023833066;
K_lean(3) = 424.726009043642;

K_steer = zeros(1,4);
K_steer(2) = 9.46120848195432;
K_steer(4) = 1.387655359381;

A_bic = A_bic - B_bic_2(:,1)*K_lean;

% K_steer = place(A_bic, B_bic, p_bic);
A_bic = A_bic - B_bic_2(:,2)*K_steer;

sys_bic_cl  = ss(A_bic, B_bic, C_bic, D_bic, ...
                               'statename', bic_states,...
                               'inputname', bic_inputs,...
                               'outputname', bic_outputs);

N_bar = dcgain(sys_bic_cl);              
tspan = 0:0.01:10;

% Initial condition : 0 rad, 0 rad/s
    % x0 = [pi/4 0]; 

x0 = [0 0 0 0]; 

u = (pi/4)*ones(size(tspan));


% /////// Simulation using ode45 //////////// %
[t, X] = ode45( @(t,X) bicycleStateDot(t, X, A_bic, B_bic, u(:,1) ), tspan, x0);

figure
plot(t, (180/pi)*X(:,1)/N_bar(1));
figure
plot(t, (180/pi)*X(:,2)/N_bar(2));
%}

%{
[y, t, x] = lsim(sys_bic_cl, u, tspan, x0);

figure
plot(t, (180/pi)*y(:,1)/N_bar(1), t, u*180/pi, ':' );
figure
plot(t, (180/pi)*y(:,2)/N_bar(2), t, u*180/pi, ':');
title('Open loop response');
xlabel('Time (sec)');
ylabel('Lean angle (deg)');

%}

                            
% isstable(sys_bic_cl)
% impulse(sys_bic_cl);
% step(sys_bic_cl);


% 
% format long g
% K_bic = place(A_bic, B_bic, p_bic);
% K_bic = [K_bic(1,1) 0 K_bic(1,3) 0;
%          0 0 0 0];
% 


%               
% N_bar = dcgain(sys_bic_cl);         
% t = 0:0.01:4;
% step(sys_bic_cl);
% step(sys_bic_cl(1)/N_bar(1), sys_bic_cl(2)/N_bar(2) );