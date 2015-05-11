function out = set_ylim(ax,varargin)


    lines = findobj(ax,'type','line');
    % assert(~isempty(lines),'No lines in axes to scale ylim on');

    % get max/min of first line
    ydata = get(lines(1),'ydata');
    ymin = min(ydata);
    ymax = max(ydata);

    % iterate over remaining lines
    n = length(lines);
    if n>1
        for i=2:n
            ydata = get(lines(i),'ydata');
            ymax_tmp = max(ydata);
            ymax = max(ymax,ymax_tmp);
            ymin_tmp = min(ydata);
            ymin = min(ymin,ymin_tmp);
        end
    end
    
    % ylimits
    out = [ymin,ymax];
    
    
    % incorperate margin
    range = diff(out);
    margin = 0.05*range;
    if nargin>1, margin = varargin{1}*range; end;
    
    
    set(ax,'ylim',[ymin-margin,ymax+margin]);
end
