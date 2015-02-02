function Cgain = C_gain(sys,freq)
%C_GAIN, computes gain for desired bandwidth
%   C_GAIN(sys,freq), sys is the system, freq is the desired frequencie
%   in Hz.

    Cgain = 1/abs(freqresp(sys,freq*2*pi));
end