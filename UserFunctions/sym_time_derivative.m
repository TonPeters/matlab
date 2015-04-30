function out = sym_time_derivative(A,x,dx)
% time_derivative(A,x,dx), 
%   computest the symbolic time derivative of the matrix A.
%       x contains the time dependant variables
%       dx contains the symbolic time derivatives of x

    [n_row,n_col] = size(A);
%     n_x = length(x);
    
    dA = zeros(n_row,n_col);
    dA = sym(dA);
%     dA = sym('dA',[n_row,n_col]);
    for i=1:1:n_row
        for j = 1:1:n_col
            dA(i,j) = sym_partial_derivative(A(i,j),x)*dx;
%             for k = 1:1:n_x
%                 dA(i,j) = dA(i,j)+diff(A(i,j),x(k))*dx(k);
%             end
        end
    end
    
    out = dA;
    
end