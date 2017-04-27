function simulateTrajectory(Yt)
%SIMULATE_TRAJECTORY Summary of this function goes here
%   Detailed explanation goes here

persistent x_lim x_lim_m y_lim y_lim_m;

x = Yt(7);
y = Yt(8);
delta = Yt(2);
psi = Yt(6);

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
% rectangle('Position',[w1x,w1y,wr,wr],'Curvature',1,'FaceColor',[1 1 1],'EdgeColor',[1 1 1])
% rectangle('Position',[w2x,w2y,wr,wr],'Curvature',1,'FaceColor',[1 1 1],'EdgeColor',[1 1 1])

plot([x px],[y py],'w','LineWidth',2)
hold on
plot([px sx],[py sy],'w','LineWidth',2)

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

