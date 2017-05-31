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
v = 1;

% Balancing mecha. Spring const. and damping const.
bic_states = {'phi' 'delta' 'phi_dot' 'delta_dot'};
bic_inputs = {'leanT' 'steerT'};
bic_outputs = {'phi','steer'};

A = getStateTransitionMatrix(v, bic.B, 0);
B = bic.B; 
C = bic.C;
D = bic.D;

G = ss(A, B, C, D);

f1 = figure(1);
clf;
w = logspace(-4,2,120);
sigma(G, w); grid;
text(1e0, -20,'\sigma_{max}(S)');
text(1e0, -55, '\sigma_{min}(S)');
xlabel('Frequency')
ylabel('Singular values')
title('Singular value plot of the bicycle model')
legend('Nominal system')


%
% performance weights
%---------------------

%  Wp
%    1
A = 0.1*10.0^(-2); %
wb = 10;
M =  1.05;

nuWp = [1/M  wb];     % 
dnWp = [1.0  wb*A];
gainWp = 0.5*10^(0); % 
Wp11 = gainWp*tf(nuWp,dnWp);
%---------------------
%    2
A = 0.1*10.0^(-2); %
wb = 10;
M = 1.05;

nuWp = [1/M  wb];     %     
dnWp = [1.0  wb*A];
gainWp = 0.5*10^(0); % 
Wp22 = gainWp*tf(nuWp,dnWp);
%----------------------------
Wp = [Wp11 0      ;
            0       Wp22];
        
% Wu
% Wu = eye(2);

[k, cl, gam] = mixsyn(G, Wp, [], []);
gam
get(k)
% cl_poles = poles(cl)

looptransfer = loopsens(G, k);

L = looptransfer.Lo;
T = looptransfer.To;
I = eye(size(L));

f2 = figure(2);
omega = logspace(-1,3, 100);
sigma(1+L, 'b-', Wp/gam, 'r--', T, 'b-.', omega)
grid

legend('1/\sigma(S) performance', ...
             '\sigma(Wp) performance bound', ...
             '\sigma(T) robustness');
         
f3 = figure(3);
omega = logspace(-1,3,100);
sigma(L, 'b-', Wp/gam, 'r--', omega)
grid

legend('\sigma(L)', '\sigma(Wp)  performance bound ');

figure(3);
omega = logspace(-1,3,100);
sigma(k,omega)
grid
title('Singular values of K')

%
% Transient responses of the system

f4 = transientResponse(T, 4, 100);

saveas(f1, 'figures/Hinfinity/mixSensitivity/sigmaG.png');
saveas(f2, 'figures/Hinfinity/mixSensitivity/sing1.png');
saveas(f3, 'figures/Hinfinity/mixSensitivity/sing2.png');
saveas(f4, 'figures/Hinfinity/mixSensitivity/response.png');

%{
% Wu

%-------------------------------------
figure(2)
bodemag(1/Wp11,'r-',1/Wp44,'b--',1/Wp55,'c-.',{10^(-3) 10^2})
grid
title('Inverse performance weighting functions')
legend('W_{p11}^{-1}','W_{p44}^{-1}','W_{p55}^{-1}')
%
% Noise shaping filters
Wn11 = 0.1*tf([1 2], [0.001 1]);  % 
Wn22 = 1.0*tf([1 2], [0.001 1]);  % 
%-------------------------------------
Wn = [Wn11  0
            0   Wn22];
%-------------------------------------
figure(3)
bodemag(Wn11,'r-',Wn22,'b--',{10^(-1) 10^4})
grid
title('Sensor noise weight')
legend('Wn11','Wn22')
%
% control action weights
nuWu = [1.2    1];    % 
dnWu = [0.0024  1];   % 
gainWu = 2.0*10^(-2); %
Wu11 = gainWu*tf(nuWu,dnWu);
%-------------------------------------
Wu = [Wu11 0
       0 Wu11];
%------------------------------------- 
figure(4)
bodemag(1/Wu11,'r-',{10^(-1) 10^4})
grid
title('Inverse control weighting functions')
%
s = tf('s');
Intg = 1/(s+10^(-6)); 
%
% open-loop connection with the weighting functions
% 2 dof controller
%         
systemnames    = ' G_unc Wn Wp Wu Intg ';
inputvar       = '[ ref{4}; noise{2}; control{2} ]';
outputvar      = '[ Wp; Wu; ref; -G_unc(1:2); -G_unc(3:4)-Wn; Intg ]';
input_to_G_unc = '[ control ]';
input_to_Wn    = '[ noise ]';
input_to_Wp    = '[ ref(1:4)-G_unc(1:4); Intg ]';  
input_to_Wu    = '[ control ]';
input_to_Intg  = '[ ref(1)-G_unc(1) ]';
sys_ic         = sysic;
%
C_hh = [1 0];
%} 

%{ 
%
% performance weights
%---------------------
%    1
tol = 0.6*10.0^(-1); % 
nuWp = [0.4  1];     % 
dnWp = [5.0  tol];
gainWp = 0.95*10^(0); % 
Wp11 = gainWp*tf(nuWp,dnWp);
%---------------------
%    2
Wp22 = 0.93; %           
%---------------------
%    3
Wp33 = 0.15; % 
%---------------------
%    4
nuWp = [1.1 1]; 
dnWp = [1.0 1];
gainWp = 0.22*10^(0); % 
Wp44 = gainWp*tf(nuWp,dnWp);
%---------------------
%    5
tol = 0.6*10.0^(-1); %
nuWp = [0.4  1];     % 
dnWp = [5.0  tol];   %
gainWp = 1.8*10^(0); % 
Wp55 = gainWp*tf(nuWp,dnWp);
%----------------------------
Wp = [Wp11 0          0   0         0;
            0       Wp22    0   0         0;
            0       0   Wp33    0         0;
            0       0     0         Wp44  0;
            0       0     0         0   Wp55];
%-------------------------------------
figure(2)
bodemag(1/Wp11,'r-',1/Wp44,'b--',1/Wp55,'c-.',{10^(-3) 10^2})
grid
title('Inverse performance weighting functions')
legend('W_{p11}^{-1}','W_{p44}^{-1}','W_{p55}^{-1}')
%
% Noise shaping filters
Wn11 = 0.1*tf([1 2], [0.001 1]);  % 
Wn22 = 1.0*tf([1 2], [0.001 1]);  % 
%-------------------------------------
Wn = [Wn11  0
       0   Wn22];
%-------------------------------------
figure(3)
bodemag(Wn11,'r-',Wn22,'b--',{10^(-1) 10^4})
grid
title('Sensor noise weight')
legend('Wn11','Wn22')
%
% control action weights
nuWu = [1.2    1];    % 
dnWu = [0.0024  1];   % 
gainWu = 2.0*10^(-2); %
Wu11 = gainWu*tf(nuWu,dnWu);
%-------------------------------------
Wu = [Wu11 0
       0 Wu11];
%------------------------------------- 
figure(4)
bodemag(1/Wu11,'r-',{10^(-1) 10^4})
grid
title('Inverse control weighting functions')
%
s = tf('s');
Intg = 1/(s+10^(-6)); 
%
% open-loop connection with the weighting functions
% 2 dof controller
%         
systemnames    = ' G_unc Wn Wp Wu Intg ';
inputvar       = '[ ref{4}; noise{2}; control{2} ]';
outputvar      = '[ Wp; Wu; ref; -G_unc(1:2); -G_unc(3:4)-Wn; Intg ]';
input_to_G_unc = '[ control ]';
input_to_Wn    = '[ noise ]';
input_to_Wp    = '[ ref(1:4)-G_unc(1:4); Intg ]';  
input_to_Wu    = '[ control ]';
input_to_Intg  = '[ ref(1)-G_unc(1) ]';
sys_ic         = sysic;
%
C_hh = [1 0];
%}