%{
x = -4:4;
y = [0 .15 1.12 2.36 2.36 1.46 .49 .06 0];
cs = spline(x,[0 y 0]);
xx = linspace(-4,4,101);
plot(x,y,'o',xx,ppval(cs,xx),'-');
%}


%{
x = pi*[0:.5:2]; 
y = [0  1  0 -1  0  1  0; 
     1  0  1  0 -1  0  1];
pp = spline(x,y);
yy = ppval(pp, linspace(0,2*pi,101));
plot(yy(1,:),yy(2,:),'-b',y(1,2:5),y(2,2:5),'or'), axis equal
%}

%{
x = -3:3; 
y = [-1 -1 -1 0 1 1 1]; 
xq1 = -3:.01:3;
p = pchip(x,y,xq1);
s = spline(x,y,xq1);
plot(x,y,'o',xq1,p,'-',xq1,s,'-.')
legend('Sample Points','pchip','spline','Location','SouthEast')
%}

%{
x = 0:25;
y = besselj(1,x);
xq2 = 0:0.01:25;
p = pchip(x,y,xq2);
s = spline(x,y,xq2);
plot(x,y,'o',xq2,p,'-',xq2,s,'-.')
legend('Sample Points','pchip','spline')
%}

% h = ezplot('x^4 + y^3 = 2*x*y');

%{
p2 = 3/8*pi;
p3 = -1/(16*pi^2);
ti = 0;
tf = 12.576;
dt = 0.016;
t = [ti:dt:tf]';
gamma = polyval([p3 p2 0 0],t);
a = 0.0625;
b = 0.25;
x = (b-a)*sin(t)+3*a*sin((t/a)*(b-a));
z = 0.5+(b-a)*cos(t)-3*a*cos((t/a)*(b-a));
plot(t,gamma),grid;
%}