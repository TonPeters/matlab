function legend_outside(fig,leg_list)
% legend_outside(fig,leg_list), inputs are the figure handle and the list
% containing all legend entries. The legend is placed on the northeast side
% outside of the subplots and all subplots are scaled accordingly. Only
% works with a row of plots.

    ax = findobj(fig,'type','axes','tag','');
    
    % determine wich figure is on top
    pos_start = get(ax(1),'position');
    pos_end = get(ax(end),'position');
    if pos_start(2)>pos_end(2)
        % plot legend in the top axes
        axes(ax(1));
        legend(leg_list,'location','NorthEastOutside');
        % get the new axes width
        pos = get(ax(1),'position');
        width = pos(3);
    else
        % plot legend in the top axes
        axes(ax(end));
        legend(leg_list,'location','NorthEastOutside');
        % get the new axes width
        pos = get(ax(end),'position');
        width = pos(3);
    end
    
    
    % Set all subplot widths accordingly
    for i=1:1:length(ax)
        pi = get(ax(i),'position');
        pi(3) = width;
        set(ax(i),'position',pi);
    end
    
end