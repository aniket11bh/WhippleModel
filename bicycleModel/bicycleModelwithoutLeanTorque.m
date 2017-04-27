% This file basically combines leanController file and steerController file
% and use their gains with state of 

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

v = 4;
C = v*C1;
K = g*K0 + v*v*K2;

S1 = -invM*K;
S2 = -invM*C;


A_bic = [ 0 0 1 0 ;
                  0 0 0 1 ;
                  S1  S2 ];

B_bic = [0
                0
                invM(2,1)
                invM(2,2)];
 
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
% p_bic = [-6+8i , -6-8i, -100, -200];

K_roll = [7843.20023833066 0 424.726009043642 0;
                0 0 0 0];
% K_steer= [0 30.2760388726693 0 2.77531071876199;
%                     0 0 0 0];
K_steer= [0  56.8966724666967 0 7.16138850467022;
                    0 0 0 0];
                
a = [0 0; 0 0; b31 0; 0 0]*K_roll;
b = [0 0; 0 0; 0 0; b42 0]*K_steer;
A_bic = A_bic -a;

% 
% format long g
% K_bic = place(A_bic, B_bic, p_bic);
% K_bic = [K_bic(1,1) 0 K_bic(1,3) 0;
%          0 0 0 0];
% 

sys_bic_cl  = ss(( A_bic), B_bic, C_bic, D_bic, ...
                                'statename', bic_states,...
                                'inputname', bic_inputs,...
                                'outputname', bic_outputs);
              
N_bar = dcgain(sys_bic_cl);         
t = 0:0.01:4;
step(sys_bic_cl);
% figure, step(sys_bic_cl(1)/N_bar(1))
% figure, step(sys_bic_cl(2)/N_bar(2) );