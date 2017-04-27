function sketch(cmd)
if nargin == 0
    cmd = 'init';
end

switch cmd
case 'init'
    fig = figure('DoubleBuffer','on','back','off');
    info.ax = axes('XLim',[0 1],'YLim',[0 1]);
    info.drawing = [];
    info.x = [];
    info.y = [];
    set(fig,'UserData',info,...
            'WindowButtonDownFcn',[mfilename,' down'])
    
case 'down'
    myname = mfilename;
    fig = gcbf;
    info = get(fig,'UserData');
    curpos = get(info.ax,'CurrentPoint');
    info.x = curpos(1,1);
    info.y = curpos(1,2);
    info.drawing = line(info.x,info.y,'Color','k');
    set(fig,'UserData',info,...
            'WindowButtonMotionFcn',[myname,' move'],...
            'WindowButtonUpFcn',[myname,' up'])
    
case 'move'
    fig = gcbf;
    info = get(fig,'UserData');
    curpos = get(info.ax,'CurrentPoint');
    info.x = [info.x;curpos(1,1)];
    info.y = [info.y;curpos(1,2)];
    set(info.drawing,'XData',info.x,'YData',info.y)
    set(fig,'UserData',info)
    
case 'up'
    fig = gcbf;
    set(fig,'WindowButtonMotionFcn','',...
            'WindowButtonUpFcn','')
    
end