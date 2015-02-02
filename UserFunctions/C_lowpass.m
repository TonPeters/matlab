function Clp = C_lowpass(flp,order,damp,varargin)
%C_LOWPASS, computes low-pass filter at freq flp
% Clp = w^order/(s+w)^order
% 
%   C_LOWPASS(flp) gives an low pass with a slope -2 after the defined
%   frequency flp.
% 
%   C_INT(flp,order), possible to change the order (default = 2).
% 
%   C_INT(flp,order,s), s is the tf variable (default = s).

    if nargin>3,
        error('In save_figure.m, \nTo many input arguments (max=3): \n\tnargin = %1.0f',nargin); 
    elseif nargin<2
        order = 2;
        damp = 0.7;
        s = tf('s');
    elseif nargin<3;
        damp = 0.7;
        s = tf('s');
    elseif nargin<4;
        s = tf('s');
    else
        s = varargin;
    end
    
    w = flp*2*pi;
    if order==1
        Clp = w^order/(s+w)^order;
    elseif order ==2
        Clp = w^order/(s^2+2*w*damp*s+w^2);
    else
        disp('wrong order');
        Clp = false;
    end
        
end