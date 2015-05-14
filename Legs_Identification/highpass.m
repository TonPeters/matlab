function [y,N,B,A]= highpass(x,fs,fc);
% Description: This function uses an elliptical filter
% to filter out high frequencies.
%
% [y,N,B,A] = highpass(x,fs,fc)
%
% Variable Definition:
% x: Input signal
% fs: Input signal sampling frequency in Hz
% fc: cutoff frequency
%
% y: Filtered signal (output)
% N: Filter order
% B: Feedforward (FIR) coefficients
% A: Feedback (IIR) coefficients
%
% Copied from: http://jeastham.blogspot.nl/2014/05/implementing-filters-in-matlab-using.html
Rp = 0.5;   % Ripple in dB (passband)
Rs = 20;    % Attenuation
mf = fs/2;  % Maximum frequency
Wp = fc/mf; % Cuttoff frequency
Ws = Wp/1.2; %Stopband frequency
% First, determine the minimum filter order to meet specifics
[N,Wn] = ellipord(Ws,Wp,Rp,Rs);
% Next, Design the filter (find the coefficients)
[B,A] = ellip(N,Rp,Rs,Wn,'high');
% Now, apply the filter (filter the signal)

% Check for leading and trailing NaNs
si = 1; % Startindex
while ( isnan(x(si)) );
    si = si+1;
end
ei = length(x); % End index
while ( isnan(x(ei)) );
    ei = ei -1;
end
fprintf('Length = %i, start = %i, end = %i\n', length(x), si, ei)

% Filter stuff
yf = filtfilt(B,A,x(si:ei));

% Put filtered values in output
y = zeros(size(x));
y(si:ei) = yf;

% Plot the filter
%figure('Color',[1 1 1]);
%freqz(B,A,2^14,fs);