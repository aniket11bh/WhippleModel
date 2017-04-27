function [G] = findGoal(irel, eps, Q1, Q2, Dap, L, X, lookAheadDist, Z)
%findGoal finds goal point for the controller
% Inputs :
%       inode : current node
%       eps : cross talk error
%       Q1 : i to x
%       Q2 : x to i+1
%       Dap : 
%       X : vector of waypoints
%       lookAheadDist :


    temp = (norm(Q1) >= lookAheadDist) && (norm(Q2) >= lookAheadDist);
    Lp = norm(L);
    inode = irel;
    
    if isempty(Z)
        iX = X(:,irel);
        iX_plus1 = X(:,irel+1);
    else
        iX = Z(:,1);
        iX_plus1 = Z(:,2);
    end
    
    fprintf('\tfindGoal : ');
    
    if irel == 1 && temp
        fprintf('Infront of first node \n');
        G = iX;
    end
    
    if (irel == 1 && ~temp) || irel ~= 1
        if (norm(Q1) <= lookAheadDist) && (norm(Q2) >= lookAheadDist)
            % case 2 (1 intersection point at ith relv. segment)
            fprintf('1 pt intersection with segment: %d \n',irel);
            cosgamma = (norm(Q2)*norm(Q2) - norm(Q1)*norm(Q1) - Lp*Lp)/(-2*Lp*norm(Q1));
            P = norm(Q1)*cosgamma + sqrt(norm(Q1)*norm(Q1)*(cosgamma*cosgamma - 1) + lookAheadDist*lookAheadDist);
            G = P*[(iX_plus1(1) - iX(1))/Lp
                         (iX_plus1(2) - iX(2))/Lp];
            G = G + iX;
            
        else
            if inode == irel && (norm(Q1) >= lookAheadDist) && (norm(Q2) >= lookAheadDist)
                if abs(eps) <= lookAheadDist
                    % case 3 (2 intersection point at ith relv. segment)
                    fprintf('2 pt intersection with segment %d\n', irel);

                    P = sqrt((lookAheadDist*lookAheadDist) - (eps*eps));
                    G = P*[(iX_plus1(1) - iX(1))/Lp
                                 (iX_plus1(2) - iX(2))/Lp];
                    G = G + L;
                    
                else
                    % case 4, (No intersection point with ith relv.
                    % case 5 (No intersection point with dead zone)                    
                    fprintf('0 pt intersection, segment %d \n', irel);
                    G = Dap*[iX(1)/Lp
                                      iX(2)/Lp];
                    G = G + iX;
                end
            elseif  inode == irel && (norm(Q1) <= lookAheadDist) && (norm(Q2) <= lookAheadDist)
                 fprintf('Special, 0 pt intersection, segment %d \n', irel);
                    G = iX_plus1;
            elseif (norm(Q1) >= lookAheadDist) && (norm(Q2) <= lookAheadDist)
                    G = iX_plus1;
            else
                % move to next segment
                fprintf('Moving to next segment\n');
                G = [];
            end
        end
    end
end

