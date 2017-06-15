function simulateTrajectoryControl(X, Yt, G, La)
%SIMULATE_TRAJECTORY_CONTROL Summary of this function goes here
%   Detailed explanation goes here

global TrajectoryFlag
global bic

persistent x_lim x_lim_m y_lim y_lim_m;

x = Yt(7);
y = Yt(8);
delta = Yt(3);
psi = Yt(6);

% Trajectory points
xq = X(1,:);
yq = X(2,:);

% Test
% x = 0;
% y = 0;
% delta = pi/4;
% psi = pi/4;

% dimensions
R = 0.1;
w = 1.0;

% positions of front wheel contact point
px = x + bic.w*cos(psi);
py = y + bic.w*sin(psi);

% Lookahead distance circle
l_theta = 0:0.01:2*pi;
lx = x + La*cos(l_theta);
ly = y + La*sin(l_theta);

% steering angle line repre.
sx = px + 0.25*cos(delta+psi);
sy = py + 0.25*sin(delta+psi);

plot(lx, ly,':','LineWidth',1); % lookahead dist circle
hold on
rectangle('Position',[x-R/2,y-R/2,R, R],'Curvature',1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1]) % rear wheel circle
rectangle('Position',[px-R/2,py-R/2,R, R],'Curvature',1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1]) % front wheel circle
plot([x px],[y py],'w','LineWidth',2) % body
plot([px sx],[py sy],'w','LineWidth',2) % steering

axis([-0.5 2.5 -0.5 2.5]);
plot(xq,yq,':.');

% Draw goal vector arrow
if strcmp(TrajectoryFlag, 'search')
    quiver(x,y,G(1)-x,G(2)-y,0, 'MaxHeadSize',0.5)
end

% set(gca,'YTick',[])
% set(gca,'XTick',[])


% if x < w + 0.5
%      x_lim_m = -(abs(x) + w + 0.5);
% end
%     
% if (abs(x) + w + 0.25) > 5
%     x_lim = abs(x) + w + 0.5;
% else
%      x_lim = 5;
%      x_lim_m = 0;
% end
% 
% if y < w + 0.5
%      y_lim_m = -(abs(y) + w + 0.5);
% end
%     
% if (abs(y) + w + 0.25) > 2.5
%     y_lim = abs(y) + w + 0.5;
% else
%      y_lim = 2.5;
%      y_lim_m = 0;
% end

x_lim_m = x-2;
x_lim = x+2;
y_lim_m = y - 2; 
y_lim = y+2;

xlim([ x_lim_m x_lim]);
ylim([ y_lim_m y_lim]);
set(gca,'Color','k','XColor','w','YColor','w')
set(gcf,'Position',[10 900 800 400])
set(gcf,'Color','k')
set(gcf,'InvertHardcopy','off')

box off
drawnow
hold off



end



