function scr(mode,fig)
if(nargin<2),fig=gcf;end;
%screen info
w=1920; h=1200; 
ofset = 0; startbalk = 68; menubalk = 42;
if(evalin('caller','exist(''onescreen'')'))%check for existence of onescreen variable (can be global)    
    x1=-1671; y1=9;   w1=1664; h1=946; 
    x2=-831;  y2=534; w2=824;  h2=422; 
else
%     w1 = w-2*ofset;                     w2 = w/2-2*ofset; 
%     x1 = ofset;                         x2 = w/2+ofset;
%     h1 = h-startbalk-ofset-menubalk;    h2 = (h1-menubalk-ofset)/2;
%     y1 = startbalk+ofset;               y2 = y1+h2+ofset+menubalk;   
    x1 = 75; y1 = 35; w1 = w-startbalk-10; h1 = h-menubalk-10;
    x2 = w1/2+startbalk+5; y2 = h/2+menubalk-10; w2 = w1/2-10; h2 = y2-2*menubalk-10;
end

switch mode
    case 'm' %maximize
        set(fig,'outerposition',[x1 y1 w1 h1]); 
    case 'l' %left      
        set(fig,'outerposition',[x1 y1 w2 h1]);      
    case 'r' %right
        set(fig,'outerposition',[x2 y1 w2 h1]);      
    case 't' %top
        set(fig,'outerposition',[x1 y2 w1 h2]);      
    case 'b' %bottom
        set(fig,'outerposition',[x1 y1 w1 h2]);      
    case 'lt' %lefttop
        set(fig,'outerposition',[x1 y2 w2 h2]);      
    case 'lb' %leftbottom
        set(fig,'outerposition',[x1 y1 w2 h2]);      
    case 'rt' %righttop
       set(fig,'outerposition',[x2 y2 w2 h2]);       
    case 'rb' %rightbottom
        set(fig,'outerposition',[x2 y1 w2 h2]);       
end
shg;
        