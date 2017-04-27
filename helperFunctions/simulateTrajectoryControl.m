function simulateTrajectoryControl(X, Yt, G, La)
%SIMULATE_TRAJECTORY_CONTROL Summary of this function goes here
%   Detailed explanation goes here

persistent x_lim x_lim_m y_lim y_lim_m;

x = Yt(7);
y = Yt(8);
delta = Yt(2);
psi = Yt(6);

% Trajectory points
xq = X(1,:);
yq = X(2,:);
% points = [1.5 1; 3.5 1; 3.5 3; 1.5 3; 1.5 5; 3.5 5];
points = [1.5 1; 2.5 1; 3.5 1; 4.5 1; 6.5 1; 8 1];
xt = points(:,1);
yt = points(:,2);

% Test
% x = 0;
% y = 0;
% delta = pi/4;
% psi = pi/4;

% dimensions
% L = 2;  % pendulum length
R = 0.1;

w = 1.0;

% positions
px = x + w*cos(psi);
py = y + w*sin(psi);

sx = px + 0.25*cos(delta+psi);
sy = py + 0.25*sin(delta+psi);

plot([-10 10],[0 0],'w','LineWidth',2)
hold on
rectangle('Position',[x-R/2,y-R/2,R, R],'Curvature',1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1])
rectangle('Position',[px-R/2,py-R/2,R, R],'Curvature',1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1])
% rectangle('Position',[x-La/2,y-La/2,La, La],'Curvature',1,'FaceColor',[0 0 0],'EdgeColor',[1 1 1])
% scatter(x,y,La,'filled','MarkerFaceAlpha',3/8,'MarkerFaceColor',[1 0.1 0.1])    

plot([x px],[y py],'w','LineWidth',2)
hold on
plot([px sx],[py sy],'w','LineWidth',2)
plot(xt,yt,xt,yt,'o');
axis([-0.5 2.5 -0.5 2.5]);
plot(xt,yt,'o',xq,yq,':.');

quiver(x,y,G(1)-x,G(2)-y,0, 'MaxHeadSize',0.5)

% rectangle('Position',[px-mr/2,py-mr/2,mr,mr],'Curvature',1,'FaceColor',[.3 0.3 1],'EdgeColor',[1 1 1])

% set(gca,'YTick',[])
% set(gca,'XTick',[])


if x < w + 0.5
     x_lim_m = -(abs(x) + w + 0.5);
end
    
if (abs(x) + w + 0.25) > 5
    x_lim = abs(x) + w + 0.5;
else
     x_lim = 5;
     x_lim_m = 0;
end

if y < w + 0.5
     y_lim_m = -(abs(y) + w + 0.5);
end
    
if (abs(y) + w + 0.25) > 2.5
    y_lim = abs(y) + w + 0.5;
else
     y_lim = 2.5;
     y_lim_m = 0;
end


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



