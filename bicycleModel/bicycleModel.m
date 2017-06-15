clear all
% close all
clc

% Bicycle model

% Load Bicycle parameters
addpath('params');
addpath('helperFunctions');

bicycleParam;
dcMotorParam;

% Bicycle velocity
v = 1;

% Balancing mecha. Spring const. and damping const.
bic_states = {'phi' 'delta' 'phi_dot' 'delta_dot'};
bic_inputs = {'leanT' 'steerT'};
bic_outputs = {'phi','steer'};

A = getStateTransitionMatrix(v, bic.B, 3);
B = bic.B; 
C = bic.C;
D = bic.D;

%%%%%%%%%% Pole placement %%%%%%%%%%%%%%
% Design req. :  for steer
%               Mp < 0.1 rad
%               tr < 1 sec
%           zeta = 0.6, wn = 10
%           p_steer = [-3+1i , -3-1i];
% 
% Design req. : for Lean
%               Mp < 0.1 rad
%               tr < 1 sec
%            zeta = 0.3152, wn = 10.75
%           p_lean = [-3.384+10.2i , -3.384-10.2i];

% Use k from lean and steer controller
%{
K_lean = zeros(1,4);
K_lean(1) = 7843.20023833066;
K_lean(3) = 424.726009043642;

K_steer = zeros(1,4);
K_steer(2) = 33.0078;
K_steer(4) = 5.4516;
% eig(A)
% A = A - B(:,1)*K_lean - B(:,2)*K_steer;
% eig(A)
%}

%{
poles = [-3+1i, -3-1i, -3.384+10.2i , -3.384-10i];
format long;
k = real(place(A, B, poles));
K = zeros(2,4);
K(1,:) = [k(1,1) 0 k(1,3) 0];
K(2,:) = [0 k(2,2) 0 k(2,4)];
%}

% LQR
% Q = C'*C;
% R = eye(2,2);
% K = lqr(A,B,Q,R)

sys_bic = ss(A, B, C, D, ...
                     'statename', bic_states,...
                     'inputname', bic_inputs,...
                     'outputname', bic_outputs);
% syms s;
% de=det(s*eye(4)-A);
% de2=collect(de,s)

N_bar = dcgain(sys_bic);   

sys_bic_norm  = ss(A, B/N_bar, C, D, ...
                          'statename', bic_states,...
                          'inputname', bic_inputs,...
                          'outputname', bic_outputs);
                      

x0 = [0 0 0 0]; 

% For own effect and coupling effects
% transientResponse(sys_bic_cl, 4, 100);

% %{ 

t = 0:0.01:4;
% impulse(sys_bic_cl);

% Reference var.
r = [0*ones(size(t))
       pi/4*ones(size(t))];

[y, t, x] = lsim(sys_bic_norm, r, t, x0);
yyaxis left
plot(t, y(:,1)*180/pi);
hold on
plot(t, r(1,:)*180/pi,':');
ylabel('Lean angle (degrees)')

yyaxis right
plot(t, y(:,2)*180/pi);
plot(t, r(2,:)*180/pi, ':');
ylabel('Steer angle (degrees)');

hold off;
title('Step Response of steer and lean angles')
xlabel('time (sec)')
%} 