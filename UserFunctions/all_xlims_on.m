function out = all_xlims_on()
% ALL_GRIDS_ON, turns on the grid on each open figure

    axesHandles = findobj('Type','axes','tag','');
    for i = axesHandles.'
        set_xlim(i);
    end
    out = true;
end