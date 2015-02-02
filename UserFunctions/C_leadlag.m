function Cleadlag = C_leadlag(f1,f2,varargin)
%C_LEADLAG, computes a lead lag filter
% Cll = (1/(f1*2*pi)*s+1) / (1/(f2*2*pi)*s+1)
% 
%   C_LEADLAG(f1,f2), f1 is the lead freq. and f2 the lag freq. in Hz.
%   Standard values are: f1 = fll/3, f2 = fll*3, with fll the freq. of the
%   leadlag filter.
% 
%   C_LEADLAG(f1,f2,s), s is the tf variable (default=s).

    if nargin>3,
        error('In save_figure.m, \nTo many input arguments (max=3): \n\tnargin = %1.0f',nargin); 
    elseif nargin<3
        s = tf('s');
    else
        s = varargin;
    end
    
    w1 = f1*2*pi;
    w2 = f2*2*pi;
    Cleadlag = (1/w1*s+1)/(1/w2*s+1);
end