function out = an(A,B,C)
% AN Calculates the inner angle from A->B->C in radians,
%    where A B an C are points in 2D/3D space.

    vec1 = A-B;
    vec2 = C-B;
    nvec1 = normalize(vec1);
    nvec2 = normalize(vec2);
    out = acos(nvec1.'*nvec2);

end

function v=normalize(v)
    v=v/norm(v);
end