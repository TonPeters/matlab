function scr2(mode,fig)
if(nargin<2),fig=gcf;end;
% calibration:
% yb, xl: bottem left origin of the figure
% yc, xc: center origins of the figure
% hl, wl: size of large figure
% hs, ws: size of a small figure
% First tune yb, xl, yc and xc by plotting a figure with scr m
% Second tune hl, wl, hs and ws by plotting four windows with scr (rb, rt, lb, lt) 

if(evalin('caller','exist(''twoscreen'')'))%check for existence of onescreen variable (can be global)    
    % settings
    yb = 0.03; xl = 0.04;   % left bodem
    hl = 0.94; wl = 0.95; % max size
    yc = 0.52; xc = 0.525;         % right top
    hs = 0.45; ws = 0.47;         % min size
    % position
    x1 = xl; x2 = xc; y1 = yb; y2 = yc;
    w1 = wl; w2 = ws; h1 = hl; h2 = hs; 
else
    % settings
    yb = 0.03; xl = 0.04;   % left bodem
    hl = 0.94; wl = 0.95; % max size
    yc = 0.52; xc = 0.525;         % right top
    hs = 0.45; ws = 0.47;         % min size
    % position
    x1 = xl; x2 = xc; y1 = yb; y2 = yc;
    w1 = wl; w2 = ws; h1 = hl; h2 = hs;
end

switch mode
    case 'm' %maximize
        set(fig,'units','normalized','outerposition',[x1 y1 w1 h1]); 
    case 'l' %left      
        set(fig,'units','normalized','outerposition',[x1 y1 w2 h1]);      
    case 'r' %right
        set(fig,'units','normalized','outerposition',[x2 y1 w2 h1]);      
    case 't' %top
        set(fig,'units','normalized','outerposition',[x1 y2 w1 h2]);      
    case 'b' %bottom
        set(fig,'units','normalized','outerposition',[x1 y1 w1 h2]);      
    case 'lt' %lefttop
        set(fig,'units','normalized','outerposition',[x1 y2 w2 h2]);      
    case 'lb' %leftbottom
        set(fig,'units','normalized','outerposition',[x1 y1 w2 h2]);      
    case 'rt' %righttop
       set(fig,'units','normalized','outerposition',[x2 y2 w2 h2]);       
    case 'rb' %rightbottom
        set(fig,'units','normalized','outerposition',[x2 y1 w2 h2]);       
end
shg;
        