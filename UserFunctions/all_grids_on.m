function out = all_grids_on()
% ALL_GRIDS_ON, turns on the grid on each open figure

    axesHandles = findobj('Type','axes','tag','');
    for i = axesHandles.'
        grid(i,'on')
    end
    out = true;
end