%POS   Gets the size and position of figure(fig) and store on clipboard. 
%   
%   Usage:   pos(figure)  -> standard use
%            pos figure   -> you can input the figure number as string
%            pos          -> if you don't specify the figure, all figure positions are stored
%
%            example: "pos(3)" or "pos 3" returns the message 
%            Clipboard contains: set(3,'position',[1717 532 560 420]); 
%            Paste this (CTRL-V) after your figure(3); command in your m-file. 
%            The next time you run the m-file, the figure will return to this position/size 
%   
%   Inputs:  figure number (takes current figure if called without argument)
%   
%   Outputs: stores a command on the clipboard
%   
%   See also SET
%   
%   Rob Hoogendijk (2011-08-3) 

%   Changes: 
%   2011083 - Initial version 
%   
%   ________________________________
%   Eindhoven University of Technology
%   Dept. of Mechanical Engineering
%   Control Systems Technology group
%   PO Box 513, WH -1.126
%   5600 MB Eindhoven, NL
%   T +31 (0)40 247 4227
%   F +31 (0)40 246 1418
%   E r.hoogendijk@tue.nl

function pos(fig)

%variables
str='';

%process inputs
if nargin==0
    figs=flipud(get(0,'children'));
else
    figs=fig;
    if(ischar(fig)),figs=str2num(fig);end;
end

%copy to clipboard
for ii=1:length(figs)
    fig=figs(ii);
    posvec=get(fig,'position');
    str=[str,['set(',num2str(fig),',''position'',[',num2str(posvec(1,1)),' ',num2str(posvec(1,2)),' ',num2str(posvec(1,3)),' ',num2str(posvec(1,4)),']);'  ]];
end
clipboard('copy',str);
disp(['Clipboard contains: ' clipboard('paste')]);
disp(['Paste this after your "figure(' num2str(figs(end)) ');" command in your m-file.']);

end%function
