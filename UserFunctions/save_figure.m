function save_figure(directory,filename,varargin)
%SAVE_FIGURE, saves the current figure as pdf.
%   SAVE_FIGURE(directory,filename), directory and filename are strings,
%   save the figure as a pdf in the defined directory with the defined
%   filename.
% 
%   SAVE_FIGURE(directory,filename,scale), scales the figure with the
%   double scale.
% 
%   SAVE_FIGURE(directory,filename,scale,ratio), scales the width/hight
%   ratio with the double ratio (default ratio = 6/5).

%     Check max inputs arguments
    if nargin>4, 
        error('In save_figure.m, \nTo many input arguments (max=4): \n\tnargin = %1.0f',nargin); 
    end
    
%     Check the varargin for scale and ratio
    if nargin<3, 
        scale = 1; ratio = 6/5;
    elseif nargin<4, 
        scale = varargin{1}; ratio = 6/5; 
    else
        scale = varargin{1}; ratio = varargin{2};
    end
    
%     Compute correct directory
    if isempty(directory)
        dir_file = filename;
    else
        if (directory(end) == '\' || filename(1) == '\')
            dir_file = [directory,filename];
        else
            dir_file = [directory,'\',filename];
        end
    end

%     save figure
    set(gcf,'PaperUnits', 'inches');
    set(gcf,'Papersize',[ratio*scale*5 scale*5]);
    set(gcf,'PaperPositionMode','manual');
    set(gcf,'PaperPosition',[0 0 ratio*scale*5 scale*5]);
    print(gcf,'-dpdf',dir_file)
end