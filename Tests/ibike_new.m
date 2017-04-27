M =  [ 80.81722, 2.31941332208709;
       2.31941332208709, 0.29784188199686];

K0 = [-80.95, -2.59951685249872;
      -2.59951685249872, -0.80329488458618];

K2 = [0, 76.59734589573222;
      0, 2.65431523794604];


C1 = [0     , 33.86641391492494
     -0.85035641456978 , 1.68540397397560];

g = 9.81;

vf = 10;
dv = 0.01;
num_pts = vf/dv + 1;

x = zeros(4,num_pts);
ref = zeros(1,num_pts);
V = 0:dv:vf;

for v = V

    invM = inv(M);
    K = g*K0+v*v*K2;
    S1 = -v.*invM*C1;
    S2 = -1*invM*K;

    A = [ 0 0 1 0 ;
          0 0 0 1 ;
        S2  S1 ];

    % B = [0  0;
    %      0  0;
    %      invM];

    B = [0 
         0 
        invM(1,1) 
        invM(2,2)];
 
K = [-29.6829    9.4989   -2.5265    0.1828];
% K = place(A,B,P)

    x(:,floor(v/dv)+1) = eig(A-B*K);
end

plot(V,real(x(1,:)),'b-', V,real(x(2,:)),'g-', V,real(x(3,:)),'r-', V,real(x(4,:)),'k-', V, ref,'k--');

% rank(ctrb(A, B))

% Compute the solution!
% x=[0;1;0;0]; t=0; tf=5; dt=0.01;
% 
% while (t<tf)
%     x=x+dt.*(A-B*K)*x;
%     t=t+dt;
% end;
