% This modules uses the zeta and omega_n as extracted from bicycle test,
% then uses them to pole placement 
% Finds the gain which makes the roll angle stable

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

s1 = -( invM(1,1)*K(1,1) + invM(1,2)*K(2,1));
s2 = -( invM(1,1)*C(1,1) + invM(1,2)*C(2,1));

A_lean = [0   1
                   s1 s2];

B_lean = [0
                    b31];

C_lean = [1 0];

D_lean =  0;

% Balancing mecha. Spring const. and damping const.
bic_states = {'phi' 'phi_dot'};
bic_inputs = {'leanT'};
bic_outputs = {'phi'};

%%%%%%%%%% Pole placement %%%%%%%%%%%%%%
% Design req. : 
%               Mp < 0.1 rad
%               tr < 1 sec
%            zeta = 0.3152, wn = 10.75

p_lean = [-3.384+10.2i , -3.384-10.2i];

format long g
K_lean = place(A_lean, B_lean, p_lean);

sys_lean_cl  = ss( (A_lean-B_lean*K_lean), B_lean, C_lean, D_lean, ...
                                'statename', bic_states,...
                                'inputname', bic_inputs,...
                                'outputname', bic_outputs);

N_bar = dcgain(sys_lean_cl);
t = 0:0.01:10;

% Initial condition : 0 rad, 0 rad/s
x0 = [pi/4 0]; 

u = zeros(size(t));

% [y, t, x] = lsim(sys_lean_cl/N_bar, u, t, x0);
% plot(t, y*180/pi);
% title('Open loop response');
% xlabel('Time (sec)');
% ylabel('Lean angle (deg)');

% Impulse response crosses more than 6 rad. which is bad
    impulse(sys_lean_cl/N_bar, t);

 % Step response not needed as we're not holding the bicycle at particular position   
    % step((180/pi)*sys_bic_cl/N_bar);