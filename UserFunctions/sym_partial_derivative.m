function out = sym_partial_derivative(A,x)
% time_derivative(A,x), 
%   computest the symbolic partial derivative of the matrix A to variables
%   x.

    [n_row,n_col] = size(A);
    n_x = length(x);
    assert(n_col <= 1, ...
        'Matrix A can only have one column, n_cols = %d',...
        n_col);    
    
    dA = zeros(n_row,n_x);
    dA = sym(dA);
    for i=1:1:n_row
        for k = 1:1:n_x
            dA(i,k) = diff(A(i,1),x(k));
        end
    end
    
    out = dA;
    
end