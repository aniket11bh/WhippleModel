% x_d = ;
% y_d = ;
% psi_d = ;

close all;
clear all;
clc;         

addpath('params');
addpath('helperFunctions');
addpath('bicycleModel');
bicycleParam;

global bic;

psi_d = 1;
x_d = 5;
y_d = 5;

t = 0:0.01:10;
y0 = [0; 0; 0];
u = [x_d; y_d; psi_d]*ones(size(t));

La = 1;
v = 1;

% linearized system
[A, B, C, D]  = trajectoryModel(v, 0, psi_d);

mdl_ss = ss(A, B, C, D);
G = tf(mdl_ss);

% delta = bic.W/La*(e_psi + e_y*cos(psi_d)/La - e_x*sin(psi_d)/La);
H = [tf(-La*La/(sin(psi_d)*bic.w)), tf( La*La/(cos(psi_d)*bic.w) ), tf(La/bic.w)];

L = G*H;

% L1 = G(1)*H(1);
% L2 = G(2)*H(2);
% L3 = G(3)*H(3);

T = feedback(L, eye(3));

% step(T)
lsim(T, u, t, y0);

% pzmap(T);



function [A, B, C, D] = trajectoryModel(v, delta0, psi0)
% Calculates non-linear f
% states : [x_dot, y_dot, x, y, psi]'
%   Input : v, delta0, psi0

    global bic
                
    A = [0 -v*tan(delta0)/bic.w 0 0 0;
            v*tan(delta0)/bic.w 0 0 0 0;
            1 0 0 0 0;
            0 1 0 0 0;
            0 0 0 0 0];
        
    B = [-v*v*sin(psi0)/(bic.w*cos(delta0)*cos(delta0));
             v*v*cos(psi0)/(bic.w*cos(delta0)*cos(delta0));
             0
             0
             v/bic.w];
         
    C = [0 0 1 0 0;
            0 0 0 1 0;
            0 0 0 0 1];
    
    D = [0;
            0;
            0];
        
end