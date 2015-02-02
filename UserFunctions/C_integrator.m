function Cint = C_integrator(fi,order,varargin)
%C_INTEGRATOR, computes an integrator at frequency fi
% Cint = (s+wn)^order/s^order
% 
%   C_INT(fi) gives an integrater with initial slope order 2 and changes to
%   order 0 at freq. fn in Hz.
% 
%   C_INT(fi,order), possible to change the order (default = 2).
% 
%   C_INT(fi,order,s), s is the tf variable (default = s).

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
    
    w = fi*2*pi;
    Cint = (s+w)^order/s^order;
end