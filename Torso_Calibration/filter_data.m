function out = filter_data(data,order,cut_off)
% filtfilt data using a lowpass butterword filter with cut_off frequency in 
% Hz and defined order.
    
    Fs = 1000;                          % sample frequency
    F_coff_norm = cut_off/(Fs/2);       % normalize the cut off to half the sample rate
    F_coff_norm_rad = F_coff_norm*2*pi; % translate cut off to rad/s
    
    [b_num,b_den] = butter(order,F_coff_norm_rad,'low');    % create butterworth filter
%     fvtool(b_num,b_den,'Fs',Fs,'FrequencyScale','Log')
    
    out = filtfilt(b_num,b_den,data);                       % filter data
    
end