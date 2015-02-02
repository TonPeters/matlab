function Cnotch = C_notch2(fn,dept,width,varargin)
%C_NOTCH, computes a lead lag filter 
% C_n = (s^2+wn*beta1*s+wn^2) / (s^2+wn*beta2*s+wn^2)
% 
%   C_NOTCH(fn,dept,width), fn is the frequency of the notch in Hz. 
%   dept is the dept of the notch in dB (negative means hight). width
%   determines the width of the notch (higher means wider).
% 
%   C_NOTCH(fn,dept,width,s), s is the tf variable (default=s).

    if nargin>4,
        error('In save_figure.m, \nTo many input arguments (max=3): \n\tnargin = %1.0f',nargin); 
    elseif nargin<4
        s = tf('s');
    else
        s = varargin;
    end
    
    wn = fn*2*pi;
    beta1 = width;
    beta2 = 10^(dept/20)*width;
    Cnotch = (s^2+wn*beta1*s+wn^2)/(s^2+wn*beta2*s+wn^2);
end