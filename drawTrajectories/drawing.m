function drawing
X=[];
Y=[];
h=line('parent',gca,'color','k','visible','off'); % prepare object
while 1
[x,y,b]=ginput(1);
X =[X; x];
Y=[Y;y];
set(h,'Xdata',X,'Ydata',Y,'visible','on')
if b>1
break
end
end