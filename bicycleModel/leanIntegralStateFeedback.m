% This file typically has no meaning. as integral is not present 
% in the balancing mechanism. However if present, this shows disturbance
% rejection. Hence, an implementation of integral state feedback control

% Bicycle model
clear all
close all
clc

% Load Bicycle parameters
addpath('params');
bicycleParams;

invM = inv(M);
b31 = invM(1,1);

% s = tf('s');
% G = b31/(s*s - invM(1,2)*C(2,1)*s - invM(1,1)*g*K0(1,1) - invM(1,2)*g*K0(2,1))

v = 0;
C = v*C1;
K = g*K0 + v*v*K2;

s1 = -( invM(1,1)*C(1,1) + invM(1,2)*C(2,1));
s2 = -( invM(1,1)*K(1,1) + invM(1,2)*K(2,1));

A_lean = [0   1
                    s1 s2];

B_lean = [0
                    b31];

C_lean = [1 0];

D_lean =  0;

% Balancing mecha. Spring const. and damping const.
bic_states = {'phi' 'phi_dot', 'error'};
bic_inputs = {'leanT'};
bic_inputs_dist = {'distT'};
bic_outputs = {'phi'};

%%%%%%%%%% Pole placement %%%%%%%%%%%%%%
% Design req. : 
%               Mp < 0.1 rad
%               tr < 1 sec
%           zeta = 0.3152, wn = 10.75

% p_lean_i = [-1+2i , -1-2i, -10];
p_lean_i = [-3.384+10.2i , -3.384-10.2i, -10];

% Adding integral for disturbance rejection %
A_lean_i = [A_lean [0 0]' 
                       C_lean 0 ] ;

B_lean_i = [B_lean
                     0];

B_lean_r = [0 
                       0
                     -1];

C_lean_i = [C_lean 0];

D_lean_i = D_lean;

format long g
K_lean_i = place(A_lean_i, B_lean_i, p_lean_i);


sys_bic_dist  = ss( (A_lean_i-B_lean_i*K_lean_i), B_lean_i, C_lean_i, D_lean_i, ...
                                    'statename', bic_states,...
                                    'inputname', bic_inputs_dist,...
                                     'outputname', bic_outputs);

N_bar = dcgain(sys_bic_dist);
t = 0:0.01:4;

step(sys_bic_dist);
% impulse(sys_bic_dist);