function Clp = C_highpass(flp,order,varargin)
%C_HIGHPASS, computes high-pass filter at freq flp
% Chp = w^order/(s+w)^order
% 
%   C_HIGHPASS(flp) gives an high pass with a slope -2 up to the defined
%   frequency flp.
% 
%   C_HIGHPASS(flp,order), possible to change the order (default = 2).
% 
%   C_HIGHPASS(flp,order,s), s is the tf variable (default = s).

    if nargin>3,
        error('In save_figure.m, \nTo many input arguments (max=3): \n\tnargin = %1.0f',nargin); 
    elseif nargin<2
        order = 2;
        s = tf('s');
    elseif nargin<3;
        s = tf('s');
    else
        s = varargin;
    end
    
    w = flp*2*pi;
    Clp = s^order/(s+w)^order;
end