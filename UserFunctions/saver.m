%SAVER   save a figure 'wysiwyg' (what you see is what you get)
%   
%   Usage:   saver(name,fignumber,path,filetype)
%
%   Inputs:  name:      name of the file (without extension)
%            fignumber: (optional) figure number (default: current figure)
%            path:      (optional) where to save the file (default: current dir)
%            filetype:  (optional) which filetype e.g. 'pdf', 'eps', 'png', etc. (default: '.pdf')
%   
%   Outputs: name.pdf (or different extension if chosen)
%   
%   Note:    Uses export_fig, a very powerfull figure saving function that for example
%            corrects the dashed and dotted lines that normally are very poorly saved in
%            Matlabs 'print' function.
%
%   See also SETPLOT.
   
%   Changes: 
%   20110803 - Initial version (Rob Hoogendijk)
%   20110805 - Changed check for windows/unix computer to work on linux x64 
%              as well, added default value for empty handle  (Michael
%              Ronde)
%   20110817 - Changed error to warning if chosen not to overwrite (Michael
%              Ronde)
%   20131015 - Changed input such that many different extensions can be used,
%              fixed the path input. 
%   
%   Rob Hoogendijk (2011-08-3)  
%   ________________________________
%   Eindhoven University of Technology
%   Dept. of Mechanical Engineering
%   Control Systems Technology group
%   PO Box 513, WH -1.126
%   5600 MB Eindhoven, NL
%   T +31 (0)40 247 4227
%   F +31 (0)40 246 1418
%   E r.hoogendijk@tue.nl

function saver(name,handle,path,filetype) 
%path settings: You can give your favorite paths here, I always set it to the pictures folder of my current project
%               just enter a number in the 'path' input instead of the path
preferredpath{1}='D:\Thesis\'; 
%preferredpath{2}='C:\Pictures\;
%etc,etc...'

%use current figure if no handle is supplied 
if (nargin<2) || isempty(handle)
    handle=gcf; 
end 
%default path current dir 
if (nargin<3) || isempty(path) 
    if(isunix)         
        path=[cd '/']; %for linux users 
    else 
        path=[cd '\']; %for windows users 
    end
else
    if isnumeric(path) %if path is one of the preferred paths
        path=preferredpath{path};     
    else
        %do nothing, path given in function input is the desired path
    end       
end
if (nargin<4) || isempty(filetype)
    filetype='pdf';
end


% check if the file exists
if exist([path,name,'.',filetype])
   % File does exist, ask what to do
   disp(['Overwrite file ',path,name,'.',filetype '? ']);
   R=input('[y/n]:','s');
   if(R=='n'),
       return; 
       disp('Figure not saved.'); 
   else if (R=='y')
       disp('Note: you can also hit ''Enter'' to confirm overwrite');
       end
   end       
end
    
% some settings, save fig
%path settings
currdir=cd;cd(path);
%background color
figure(handle);
currcolor=get(handle,'color');
set(handle,'color','w');
%save
export_fig([name,'.',filetype],'-nocrop');
%disp success
disp(['Figure ',num2str(handle),' stored in ',path,name,'.',filetype]);
%restore orig dir
cd(currdir);
set(handle,'color',currcolor);
end%function 
