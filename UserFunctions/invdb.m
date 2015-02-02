function out = invdb(value)
% INVDB transforms a db back to normal value
    out = 10^(value/20);
end