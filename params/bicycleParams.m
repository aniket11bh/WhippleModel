% global w c lam rR mR IRxx IRyy xB zB mB IBxx IByy IBzz IBxz ;
% global xH zH mH IHxx IHyy IHzz IHxz rF mF IFxx IFyy;

global g w invM;
global K0 K2 M C1

% geometry params
w = 1.02; % wheelbase
c = 0.08; % caster
lam = 0.314159265358979323846; % lambda in rad
g = 9.81;

% Rear wheel
rR = 0.3;
mR = 2.0;
IRxx = 0.0603;
IRyy = 0.12;

% Body
xB = 0.3;
zB = -0.9;
mB = 85.0;
IBxx = 9.2;
IByy = 11.0;
IBzz = 2.8;
IBxz = 2.4;

% Handlebar
xH = 0.9;
zH = -0.7;
mH = 4.0;
IHxx = 0.05892;
IHyy = 0.06;
IHzz = 0.00708;
IHxz = -0.00756;

% Front wheel
rF = 0.35;
mF = 3.0;
IFxx = 0.1405;
IFyy = 0.28;

%% Fillup the params basedon above params

IR = diag([IRxx IRyy IRxx]);
IF = diag([IFxx IFyy IFxx]);
IB = [IBxx 0    IBxz
      0    IByy 0
      IBxz 0    IBzz];
IH = [IHxx 0    IHxz
      0    IHyy 0
      IHxz 0    IHzz];

mT = mR + mB + mH + mF;
xT = (xB*mB + xH*mH + w*mF)/mT;
zT = (-rR*mR + zB*mB + zH*mH -rF*mF)/mT;
ITxx = IRxx + IBxx + IHxx + IFxx + mR*rR*rR + mB*zB*zB + mH*zH*zH + mF*rF*rF;
ITxz = IBxz  + IHxz - mB*xB*zB - mH*xH*zH + mF*w*rF; 
ITzz = IRxx + IBzz + IHzz + IFxx + mB*xB*xB + mH*xH*xH + mF*w*w; % IFzz = IFxx

mA = mH + mF;
xA = (xH*mH  + w*mF)/mA;
zA = (zH*mH - rF*mF)/mA;
IAxx = IHxx + IFxx + mH*(zH - zA)*(zH - zA) + mF*(rF + zA)*(rF + zA);
IAxz = IHxz -mH*(xH - xA)*(zH - zA) + mF*(w - xA)*(rF + zA);
IAzz = IHzz + IFxx + mH*(xH - xA)*(xH - xA) + mF*(w - xA)*(w - xA); % IFzz = IFxx

LAM = [sin(lam); 0; cos(lam)];
uA = (xA -w -c)*cos(lam) -zA*sin(lam);

IAll = mA*uA*uA + IAxx*sin(lam)*sin(lam) + 2*IAxz*sin(lam)*cos(lam) + IAzz*cos(lam)*cos(lam);
IAlx = -mA*uA*zA + IAxx*sin(lam) + IAxz*cos(lam);
IAlz = mA*uA*xA + IAxz*sin(lam) + IAzz*cos(lam);

mu = (c/w)*cos(lam);

sR = IRyy/rR;
sF = IFyy/rF;
sT = sR + sF;
sA = mA*uA + mu*mT*xT;

M = [ITxx            IAlx + mu*ITxz
     IAlx + mu*ITxz  IAll + 2*mu*IAlz + mu*mu*ITzz];

K0 = [mT*zT -sA
      -sA   -sA*sin(lam)];
  
K2 = [0 (sT - mT*zT)*cos(lam)/w
      0 (sA + sF*sin(lam))*cos(lam)/w];

C1 =  [0                      mu*sT + sF*cos(lam) + ITxz*cos(lam)/w - mu*mT*zT
      -(mu*sT + sF*cos(lam))  IAlz*cos(lam)/w + mu*(sA + ITzz*cos(lam)/w)];

invM = inv(M);

% v = 5;
% C = v*C1;
% K = g*K0 + v*v*K2;