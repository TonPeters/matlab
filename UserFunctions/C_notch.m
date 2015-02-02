function Cnotch = C_notch(fn,beta1,beta2,varargin)
%C_NOTCH, computes a lead lag filter 
% C_n = (s^2+wn*beta1*s+wn^2) / (s^2+wn*beta2*s+wn^2)
% 
%   C_NOTCH(fn,beta1,beta2), fn is the frequency of the notch in Hz. beta1
%   and beta2 are the damping values of the notch (should be positive). The 
%   ratio between beta1 and beta2 determines the dept/hight of the notch 
%   (beta1/beta2>1 add magnitude) and the hight of the both values 
%   determines the width of the filter (lower values result in a smaller
%   filter).
% 
%   C_NOTCH(fn,beta1,beta2,s), s is the tf variable (default=s).

    if nargin>4,
        error('In save_figure.m, \nTo many input arguments (max=3): \n\tnargin = %1.0f',nargin); 
    elseif nargin<4
        s = tf('s');
    else
        s = varargin;
    end
    
    wn = fn*2*pi;
    Cnotch = (s^2+wn*beta1*s+wn^2)/(s^2+wn*beta2*s+wn^2);
end