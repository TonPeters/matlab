function out = di(A,B)
% DI, calculates the distance between point A and B, 
%   where A and B are points in 2D/3D space.

    out = norm(A-B,2);
end