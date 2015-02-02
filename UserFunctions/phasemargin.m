function [margin,bandwidth] = phasemargin(resp)
%PHASEMARGIN, calculates the phase margin and bandwidth of a frequency 
% response dataset.
% 
%   [margin, bandwidth] = PHASEMARGIN(resp,freq), where resp is the 
%   responce data.

    bandwidth = find(db(resp)<0,1,'first');
    margin = angle(resp(bandwidth))/(2*pi)*360+180;  
end