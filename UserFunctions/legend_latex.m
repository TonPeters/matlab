function legend_latex(fig)

    leg = findobj(fig,'type','axes','tag','legend');
    
    set(leg,'interpreter','latex');
    
end