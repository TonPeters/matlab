function Cnotch = C_notch3(fn1,fn2,beta1,beta2,varargin)
%C_NOTCH, computes a notch with different frequencies
% C_n = (s^2+wn1*beta1*s+wn1^2) / (s^2+wn2*beta2*s+wn2^2)
% 
%   C_NOTCH(fn1,fn2,beta1,beta2), fn is the frequency of the notch in Hz. beta1
%   and beta2 are the damping values of the notch (should be positive). The 
%   ratio between beta1 and beta2 determines the dept/hight of the notch 
%   (beta1/beta2>1 add magnitude) and the hight of the both values 
%   determines the width of the filter (lower values result in a smaller
%   filter).
% 
%   C_NOTCH(fn1,fn2,beta1,beta2,s), s is the tf variable (default=s).

    if nargin>5,
        error('In save_figure.m, \nTo many input arguments (max=3): \n\tnargin = %1.0f',nargin); 
    elseif nargin<5
        s = tf('s');
    else
        s = varargin;
    end
    
    wn1 = fn1*2*pi;
    wn2 = fn2*2*pi;
%     Cnotch = (s^2+wn1*beta1*s+wn1^2)/(s^2+wn2*beta2*s+wn2^2);
    Cnotch = (s^2+2*wn1*beta1*s+wn1^2)/(s^2+2*wn2*beta2*s+wn2^2).*(wn2^2/wn1^2);
end