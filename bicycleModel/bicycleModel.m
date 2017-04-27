% Bicycle model

% Load Bicycle parameters
addpath('params');
bicycleParams;

invM = inv(M);
v = 4;
C = v*C1;
K = g*K0 + v*v*K2;

S1 = -invM*K;
S2 = -invM*C;


A_bic = [ 0 0 1 0 ;
                  0 0 0 1 ;
                  S1  S2 ];

B_bic = [0 0
                0 0
                invM];
 
C_bic = [1 0 0 0];

D_bic = [0 0];

% Balancing mecha. Spring const. and damping const.
bic_states = {'phi' 'delta' 'phi_dot' 'delta_dot'};
bic_inputs = {'leanT' 'steerT'};
bic_outputs = {'phi'};

%%%%%%%%%% Pole placement %%%%%%%%%%%%%%
% Design req. : 
%               Mp < 0.1 rad
%               tr < 1 sec
%           zeta = 0.6, wn = 10
% p_bic = [-6+8i , -6-8i, -100, -200];

K_roll =  [0 0 0 0; 
                  7843.20023833066 0 424.726009043642 0];
a = [0 0; 0 0; 0 B_bic(3,1); 0 0]*K_roll;
A_bic = A_bic - a;

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
              
t = 0:0.01:4;
impulse(sys_bic_cl);