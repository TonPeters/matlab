function [RGAw,RGAno] = rga(Hfrd)
% RGA = rga(Hfrd), computes the RGA value of the frequency response data
% Hfrd for each frequency
    
    no = nargout;
    
    % RGA(jw) = G x (G^-1)^T, x means element wise;
    % if RGA(G) is close to identity, decoupling is possible.
    w = Hfrd.frequency;
    nw = length(w);
    
    RGAw = zeros(size(Hfrd.response));
    RGAno = zeros(size(nw,1));
    for i = 1:1:nw
        Hfrd_wi = Hfrd.response(:,:,i);
        RGAw(:,:,i) = Hfrd_wi.*pinv(Hfrd_wi).';
        RGAno(i) = sum(sum(abs(RGAw(:,:,i)-eye(size(RGAw(:,:,i))))));
    end
    
    if no<2
        clear RGAno;
    end
end