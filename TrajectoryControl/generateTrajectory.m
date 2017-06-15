function [X] = generateTrajectory(TrajectoryFlag)
% GENERATE_TRAJECTORY generates trajectory points according 
% to it's search based or pre-plannned trajectory control

global bic;

%% Circle passing through origin
%{
R = bic.w;
xi = 1.5;
yi = 1;

xc = 1.5;
yc = yi+R;
theta = -pi/2:0.2:3*pi/2;
theta = [theta];
x = xc + R*cos(theta);
y = yc + R*sin(theta);

plot(x,y,'-x'), axis equal; grid;

X = [x; y];

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
a = 2;
e = 0.8;
b = a*sqrt(1-e*e);

xc = a*cos(pi/4);
yc = 2+b*sin(pi/4);
theta = -pi/2:0.2:3*pi/2;

x = xc + a*cos(theta);
y = yc + b*sin(theta);
z = zeros(size(x));

X = [x;
        y];
plot(x,y,'-x'), axis equal; grid;
%}

%% Rectangle
%{
points = [1.5 1; 3.5 1; 3.5 3; 1.5 3; 1.5 5; 3.5 5];
X = points';
%}

%% LaneChange
%{
x = -3:3; 
y = [-1 -1 -1 0 1 1 1]; 
xq1 = -3:.01:3;
p = pchip(x,y,xq1);
s = spline(x,y,xq1);
% plot(x,y,'o',xq1,p,'-',xq1,s,'-.')
plot(x,y,'o',xq1,p,'-')

X = [xq1;
        p];
%}


%% curve point to point
%{
x = [2 5];
y = [1 4];
xq = x(1):0.1:x(end);
yq = spline(x,[0 y 0],xq);
plot(xq,yq,':.b',x,y,'or');

X = [xq;
        yq];
%}

%% zig zag
% %{ 
t = [0 1 2 3 4 5];
points = [1.5 1; 3.5 1; 3.5 3; 1.5 3; 1.5 5; 3.5 5];
x = points(:,1);
y = points(:,2);

tq = 0:0.1:5;
slope0 = 0;
slopeF = 0;
xq = spline(t,[slope0; x; slopeF],tq);
yq = spline(t,[slope0; y; slopeF],tq);

X = [xq
        yq];
% X = points';
%}

%% Straight line
%{
points = [1.5 1; 3.5 1; 4.5 1; 5.5 1; 6.5 1; 7.5 1];

X = points';
%}
    
%%


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