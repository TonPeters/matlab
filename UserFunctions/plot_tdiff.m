function out = plot_tdiff(t,y)
% PLOT_DIFF, plots the difference
    
    td = t(2)-t(1);
    plot(t(2:end)-td/2,diff(y)/td);
    out = true;
end