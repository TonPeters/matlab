function out = save_presentation(fig,directory,filename,small)

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
    
    if(isempty(small))
        setplot(fig,[15 8]);
    else
        setplot(fig,[8 8]);
    end

    all_grids_on();

    print(fig,'-dpdf',dir_file)
end