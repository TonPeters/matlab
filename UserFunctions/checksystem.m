function checksystem(A,B,C)
% checksystem   
%   checksystem(A,B), check for controllability and stabalizable.
% 
%   checksystem(A,B,C), also check for observability and detectable.


    % Check if input arguments are vallid
    if (nargin==2 || nargin==3)
        
        % Check if C is available or not (check observability or not)
        checkOb = false;
        if nargin==3
            checkOb = true;
        end
        
        %% Check controllability and observability
        n = length(A);
        Controllable = ctrb(A,B);
        if rank(Controllable)==n
            disp('system is controllable');
        else
            disp(['system is not controllable, rank(Controllable) = ',num2str(rank(Controllable))])
        end

        if checkOb
            Observable = obsv(A,C);
            if rank(Observable)==n
                disp('system is observable');
            else
                disp(['system is not observable, rank(Observable) = ',num2str(rank(Observable))])
            end
        end

        %% Check for stabilizable and detectable
        lambda = eig(A);
        for i=1:1:length(lambda)
            if lambda(i)>0
                if rank([A-lambda(i)*eye(size(A)) B])==n
                    disp(['Unstable eigenvalue lambda',num2str(i),' = ',num2str(lambda(i)),' is stabalizable'])
                else
                    disp(['Unstable eigenvalue lambda',num2str(i),' = ',num2str(lambda(i)),' is not stabalizable'])
                end

                if checkOb
                    if rank([A-lambda(i)*eye(size(A)); C])==n
                        disp(['Unstable eigenvalue lambda',num2str(i),' = ',num2str(lambda(i)),' is detectable'])
                    else
                        disp(['Unstable eigenvalue lambda',num2str(i),' = ',num2str(lambda(i)),' is not detectable'])
                    end
                end
            end
            disp(['lambda',num2str(i),' = ',num2str(lambda(i)), ' is a stable eigenvalue']);
        end
    else
        error('improper input arguments'); 
    end
end