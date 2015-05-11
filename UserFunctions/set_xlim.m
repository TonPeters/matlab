function out = set_xlim(ax,varargin)


    lines = findobj(ax,'type','line');
    % assert(~isempty(lines),'No lines in axes to scale ylim on');

    % get max/min of first line
    xdata = get(lines(1),'xdata');
    xmin = min(xdata);
    xmax = max(xdata);

    % iterate over remaining lines
    n = length(lines);
    if n>1
        for i=2:n
            xdata = get(lines(i),'xdata');
            xmax_tmp = max(xdata);
            xmax = max(xmax,xmax_tmp);
            xmin_tmp = min(xdata);
            xmin = min(xmin,xmin_tmp);
        end
    end
    
    % ylimits
    out = [xmin,xmax];
    
    
    % incorperate margin
    range = diff(out);
    margin = 0.05*range;
    if nargin>1, margin = varargin{1}*range; end;
    
    
    set(ax,'xlim',[xmin-margin,xmax+margin]);
end
