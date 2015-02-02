function out = cosinerule(a,b,c,gamma)
%COSINE     applies the cosine rule,
%           c^2 = a^2+b^2-2abcos(gamma) and returns 
%           c or gamma depending on input.
% 
%       COSINE(a,b,c,[]), returns the angle gamma.
%       COSINE(a,b,[],gamma), returns the length of c.

    if isempty(c)
        c = sqrt(a.^2+b.^2-2.*a.*b.*cos(gamma));
        out = c;
    elseif isempty(gamma)
        gamma = acos((c.^2-a.^2-b.^2)./(-2.*a.*b));
        out = gamma;
    end
end