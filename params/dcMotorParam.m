global dcm

dcm.J = 3.2284E-6;
dcm.b = 3.5077E-6;
dcm.K = 0.0274;
dcm.R = 4;
dcm.L = 2.75E-6;

s = tf('s');
dcm.tf = dcm.K*(dcm.J*s + dcm.b)/(dcm.J*dcm.L*s*s + (dcm.J*dcm.R + dcm.b*dcm.L)*s + dcm.K*dcm.K);