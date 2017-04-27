function [X] = generateTrajectory()
%% Circle passing through origin
%{
R = 5;
xc = 3;
yc = 4;
theta = 0:0.2:2*pi;
theta = [theta 0];
x = xc + R*cos(theta);
y = yc + R*sin(theta);
z = zeros(size(x));
plot(x,y,'-x'), axis equal; grid;
%}

%% Eight
%{
x = 0:.1:2*pi;
y = -sin(x);
y1 = sin(x);
X = [x fliplr(x);
     y fliplr(y1)];
plot(X(1,:),X(2,:));
%}

%% Ellipse %
%{
a = 5;
e = 0.8;
b = a*sqrt(1-e*e);

xc = a*cos(pi/4);
yc = b*sin(pi/4);
theta = 0:0.2:2*pi;
theta = [theta 0];
x = xc + a*cos(theta);
y = yc + b*sin(theta);
z = zeros(size(x));
plot(x,y,'-x'), axis equal; grid;
%}

%% Rectangle
%{
x = 0;
y = [0 .15 1.12 2.36 2.36 1.46 .49 .06 0];
cs = spline(x,[0 y 0]);
xx = linspace(-4,4,101);
plot(x,y,'o',xx,ppval(cs,xx),'-');
%}

%% curve point to point
%{
x = [0 1];
y = [0 1];
xq = x(1):0.1:x(end);
yq = spline(x,[0 y 0],xq);
plot(xq,yq,':.b',x,y,'or');
axis([-0.5 1.5 -0.5 1.5]);
%}

%% zig zag
t = [0 1 2 3 4 5];
% points = [1.5 1; 3.5 1; 3.5 3; 1.5 3; 1.5 5; 3.5 5];
points = [1.5 1; 3 1; 4.5 1; 6 1; 7.5 1; 9 1];
x = points(:,1);
y = points(:,2);

tq = 0:0.1:5;
slope0 = 0;
slopeF = 0;
xq = spline(t,[slope0; x; slopeF],tq);
yq = spline(t,[slope0; y; slopeF],tq);

% X = [xq
%         yq];
X = points';

%{
% plot spline in t-x, t-y and x-y space
figure;
plot(t,x,t,x,'o');
axis([-0.5 5.5 -0.5 1.5]);
hold on;
plot(t,x,'o',tq,xq,':.');
title('x vs. t');

figure;
plot(t,y,t,y,'o');
hold on;
plot(t,y,'o',tq,yq,':.');
axis([-0.5 5.5 -0.5 2.5]);
title('y vs. t');
%}

 %{
figure;
plot(x,y,x,y,'o');
axis([-0.5 2.5 -0.5 2.5]);
hold on;
plot(x,y,'o',xq,yq,':.');
title('Waypoints for N-Point example');
%}
end