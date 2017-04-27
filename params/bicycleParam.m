global bic g

% geometry params
bic.w = 1.02; % bic.wheelbase
bic.c = 0.08; % caster
bic.lam = 0.314159265358979323846; % laB.mda in rad
g = 9.81;

% Rear bic.wheel
R.r = 0.3;
R.m = 2.0;
R.Ixx = 0.0603;
R.Iyy = 0.12;

% Body
B.x = 0.3;
B.z = -0.9;
B.m = 85.0;
B.Ixx = 9.2;
B.Iyy = 11.0;
B.Izz = 2.8;
B.Ixz = 2.4;

% Handlebar
H.x = 0.9;
H.z = -0.7;
H.m = 4.0;
H.Ixx = 0.05892;
H.Iyy = 0.06;
H.Izz = 0.00708;
H.Ixz = -0.00756;

% Front bic.wheel
F.r = 0.35;
F.m = 3.0;
F.Ixx = 0.1405;
F.Iyy = 0.28;

%% Fillup the params basedon above params

R.I = diag([R.Ixx R.Iyy R.Ixx]);
F.I = diag([F.Ixx F.Iyy F.Ixx]);
B.I = [B.Ixx 0    B.Ixz
            0    B.Iyy 0
            B.Ixz 0    B.Izz];
H.I = [H.Ixx 0    H.Ixz
            0    H.Iyy 0
            H.Ixz 0    H.Izz];

[bic.M, bic.K0, bic.K2, bic.C1] = getSystemMatrices(R, B, H, F, bic);
bic.invM = inv(bic.M);

% v = 5;
% C = v*C1;
% K = g*K0 + v*v*K2;

function [M, K0, K2, C1] = getSystemMatrices(R, B, H, F, bic)
        
    mT = R.m + B.m + H.m + F.m;
    xT = (B.x*B.m + H.x*H.m + bic.w*F.m)/mT;
    zT = (-R.r*R.m + B.z*B.m + H.z*H.m -F.r*F.m)/mT;
    ITxx = R.Ixx + B.Ixx + H.Ixx + F.Ixx + R.m*R.r*R.r + B.m*B.z*B.z + H.m*H.z*H.z + F.m*F.r*F.r;
    ITxz = B.Ixz  + H.Ixz - B.m*B.x*B.z - H.m*H.x*H.z + F.m*bic.w*F.r;
    ITzz = R.Ixx + B.Izz + H.Izz + F.Ixx + B.m*B.x*B.x + H.m*H.x*H.x + F.m*bic.w*bic.w; % F.Izz = F.Ixx

    mA = H.m + F.m;
    xA = (H.x*H.m  + bic.w*F.m)/mA;
    zA = (H.z*H.m - F.r*F.m)/mA;
    IAxx = H.Ixx + F.Ixx + H.m*(H.z - zA)*(H.z - zA) + F.m*(F.r + zA)*(F.r + zA);
    IAxz = H.Ixz -H.m*(H.x - xA)*(H.z - zA) + F.m*(bic.w - xA)*(F.r + zA);
    IAzz = H.Izz + F.Ixx + H.m*(H.x - xA)*(H.x - xA) + F.m*(bic.w - xA)*(bic.w - xA); % F.Izz = F.Ixx

    LAM = [sin(bic.lam); 0; cos(bic.lam)];
    uA = (xA -bic.w -bic.c)*cos(bic.lam) -zA*sin(bic.lam);

    IAll = mA*uA*uA + IAxx*sin(bic.lam)*sin(bic.lam) + 2*IAxz*sin(bic.lam)*cos(bic.lam) + IAzz*cos(bic.lam)*cos(bic.lam);
    IAlx = -mA*uA*zA + IAxx*sin(bic.lam) + IAxz*cos(bic.lam);
    IAlz = mA*uA*xA + IAxz*sin(bic.lam) + IAzz*cos(bic.lam);

    mu = (bic.c/bic.w)*cos(bic.lam);

    sR = R.Iyy/R.r;
    sF = F.Iyy/F.r;
    sT = sR + sF;
    sA = mA*uA + mu*mT*xT;

    M = [ITxx            IAlx + mu*ITxz
             IAlx + mu*ITxz  IAll + 2*mu*IAlz + mu*mu*ITzz];

    K0 = [mT*zT -sA
              -sA   -sA*sin(bic.lam)];

    K2 = [0 (sT - mT*zT)*cos(bic.lam)/bic.w
              0 (sA + sF*sin(bic.lam))*cos(bic.lam)/bic.w];

    C1 =  [0                      mu*sT + sF*cos(bic.lam) + ITxz*cos(bic.lam)/bic.w - mu*mT*zT
              -(mu*sT + sF*cos(bic.lam))  IAlz*cos(bic.lam)/bic.w + mu*(sA + ITzz*cos(bic.lam)/bic.w)];
  
end