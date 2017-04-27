close all
clear all
clc

% Read roll data from csv file
A = csvread('rolldata.csv',1);
A = A(:,1:2);

% store the variables
t = A(:,1);
phi = A(:,2);
figure
plot(t,phi);

% case 1
%{
A1 = A(A(:,1) < 12 & A(:,1) > 9.18,:);
t1 = A1(:,1);
t1 = t1 - t1(1);
ts = t1(1):0.001:t1(end);
phi1 = A1(:,2);

% fitting parameters
   omega = 10.75;
   ph    =  1.786;
   zeta  =  0.3152;
phi_fit = 43.54*exp(-zeta*omega*ts).*sin(omega*sqrt(1-zeta*zeta)*ts + ph);

figure
plot(t1, phi1);
hold on;
plot(ts, phi_fit);
I = legend('$\phi_{meas}$', '$\phi_{est}$');
set(I,'interpreter','latex');
xlabel('time (sec)');
ylabel('degrees');
hold off;
%}

% case 2
%{
A1 = A(A(:,1) < 18 & A(:,1) > 14.5,:);
t1 = A1(:,1);
t1 = t1 - t1(1);
ts = t1(1):0.001:t1(end);
phi1 = A1(:,2);

% fitting parameters
   omega = 10.75;
   ph    =  1.786;
   zeta  =  0.3152;
phi_fit = 43.54*exp(-zeta*omega*ts).*sin(omega*sqrt(1-zeta*zeta)*ts + ph);

figure
plot(t1, phi1);
hold on;
plot(ts, phi_fit);
I = legend('$\phi_{meas}$', '$\phi_{est}$');
set(I,'interpreter','latex');
xlabel('time (sec)');
ylabel('degrees');
hold off;
%}

% case 3
%{
A1 = A(A(:,1) < 23 & A(:,1) > 19.,:);
t1 = A1(:,1);
t1 = t1 - t1(1);
ts = t1(1):0.001:t1(end);
phi1 = A1(:,2);

% fitting parameters
   omega = 10.75;
   ph    =  1.786;
   zeta  =  0.3152;
phi_fit = 43.54*exp(-zeta*omega*ts).*sin(omega*sqrt(1-zeta*zeta)*ts + ph);

figure
plot(t1, phi1);
hold on;
plot(ts, phi_fit);
I = legend('$\phi_{meas}$', '$\phi_{est}$');
set(I,'interpreter','latex');
xlabel('time (sec)');
ylabel('degrees');
hold off;
%}


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
% parameter estimation using sys identification %
% u = zeros(size(t1));
% theta = findK(phi1, t1, u)

 % Parameter etimation using matrix pseudo Inverse
 function [theta] = findK(phi, t, u)
    numSamples = size(t,1);
    y = phi;
    Y = y(3:numSamples,1);
    PHI = zeros(numSamples-2,4);
 
 for k = 3:numSamples
    PHI(k-2,:) = [-y(k-1) -y(k-2) u(k-1) u(k-2)];
 end

 INV = pinv(PHI'*PHI);
 Z = (PHI'*Y);
 theta = INV*Z;
 end
 
 function [y] = f(constant, t)
    wn = constant(1);
    zeta = constant(2);
    y0 = constant(3);
    ph = constant(4);
    
%     size(wn) size(zeta) size(y0) size(ph)

    wd = wn*sqrt(1-zeta*zeta);
    p = exp(-zeta*wn*t);
    q = sin(wd*t + ph);
    y =  y0*p.*q;
 end
%}