function out = save_report(fig,directory,filename,varargin)
% save_report(fig,directory,filename), saves figure fig in directory with
%       the given filename.
% save_report(fig,directory,filename,type), define a figure type to auto
%       adjust the scale to properly fit in a report

    % Compute correct directory
    if isempty(directory)
        dir_file = filename;
    else
        if (directory(end) == '\' || filename(1) == '\')
            dir_file = [directory,filename];
        else
            dir_file = [directory,'\',filename];
        end
    end
    
    % get axes
    ax = findobj(fig,'type','axes','tag','');
    % set variables
    leg_width = [];
    fontsize = 10;
    
    width = 10;
    height = 6;   
      
    % check for plot type and adjust settings
    if(nargin>3)
        plottype = varargin{1};
        if strcmp(plottype,'bode') % single bode plot, normal size
        elseif strcmp(plottype,'bodeleg') % single bode plot, leg outside
            setplot(fig,[width height]);
            % get the legend width
            lh = findobj(fig,'Type','axes','Tag','legend');
            set(lh,'units','centimeters');
            lp = get(lh,'position');
            leg_width = lp(3)*1.2;
            width = width+leg_width;  
        elseif strcmp(plottype,'paperwidth')
            width = 12;
        elseif strcmp(plottype,'halfpaperwidth')
            width = 6*1.3;
            fontsize = 9;
        elseif strcmp(plottype,'custom') % add custom input size
            assert(nargin>4,'not enough input arguments, define size [width height]');
            width = varargin{2}(1);
            height = varargin{2}(2);            
        else
            % plot normal
            assert(false,'Incorrect input arguments');
        end
    end
    setplot(fig,[width height],{[],[],[],[],leg_width,[]},fontsize);
    
    width
    height
    all_grids_on();
    
    set(fig,'PaperUnits', 'centimeters');
    set(fig,'Papersize',[width height]);
    set(fig,'PaperPositionMode','manual');
    set(fig,'PaperPosition',[0 0 width height]);

    print(fig,'-dpdf',dir_file)
    
    
    setplot(fig,[width height],{[],[],[],[],[],[]},fontsize);
    
    out = true;
end