function [G] = findGoal(irel, eps, Q1, Q2, Dap, L, X, x, lookAheadDist, Z)
%findGoal finds goal point for the controller
% Inputs :
%       irel : relevant segment
%       eps : cross talk error
%       Q1 : i to x
%       Q2 : x to i+1
%       Dap :
%       L :
%       X : vector of waypoints
%       x : position of the bicycle
%       lookAheadDist :
%       Z : will be 0 path doesn't end i.e, inode <= numNode -1


    inode = irel; % current node for finding goal
    repeat = true; % repeat to find goal

    if Z == 0
        iX = X(:,irel);
        iX_plus1 = X(:,irel+1);
    else
        G = X(:,irel);
        return;
    end

    fprintf('\t findGoal : ');

    P  = iX_plus1 - iX;
    alpha1 = angle(Q1, P);

    if irel == 1 && (norm(Q1) >= lookAheadDist) && (norm(Q2) >= lookAheadDist) && abs(alpha1) > pi/2
        fprintf('Infront of first node');
        G = iX;
        fprintf(', Goal : %.2f, %.2f\n',G(1), G(2));
        return;
    end

    while repeat
        temp = (norm(Q1) >= lookAheadDist) && (norm(Q2) >= lookAheadDist) && abs(alpha1) > pi/2;
        P = iX_plus1- iX;
        alpha1 = angle(Q1, P);
        alpha2 = angle(Q2, P);
        Lp = norm(P);

        if (irel == 1 && ~temp) || irel ~= 1

            if (norm(Q1) <= lookAheadDist) && (norm(Q2) >= lookAheadDist)
                % case 2 (1 intersection point at ith relv. segment), common case

                fprintf('1 pt intersection with segment %d',inode);
                cosgamma = (norm(Q2)*norm(Q2) - norm(Q1)*norm(Q1) - Lp*Lp)/(-2*Lp*norm(Q1));
                P = norm(Q1)*cosgamma + sqrt(norm(Q1)*norm(Q1)*(cosgamma*cosgamma - 1) + lookAheadDist*lookAheadDist);
                G = P*[(iX_plus1(1) - iX(1))/Lp
                             (iX_plus1(2) - iX(2))/Lp];
                G = G + iX;
                repeat = false;

            else
                if inode == irel && (norm(Q1) >= lookAheadDist) && (norm(Q2) >= lookAheadDist)
                    repeat = false;

                    if abs(eps) <= lookAheadDist
                        % case 3 (2 intersection point at ith relv. segment)
                        fprintf('2 pt intersection with segment %d', irel);

                        P = sqrt((lookAheadDist*lookAheadDist) - (eps*eps));
                        G = P*[(iX_plus1(1) - iX(1))/Lp
                                     (iX_plus1(2) - iX(2))/Lp];
                        G = G + L;

                    else
                        % case 4, (No intersection point with ith relv.
                        % case 5 (No intersection point with dead zone)
                        fprintf('0 pt intersection, segment %d', irel);
%                         G = Dap*[iX(1)/Lp
%                                           iX(2)/Lp];
                        G = lookAheadDist*[(iX_plus1(1) - iX(1))/Lp
                                                             (iX_plus1(2) - iX(2))/Lp];
                        G = G + L;
                    end
                    
                else
                    % move to next segment
                    inode = inode + 1;
                    [eps, Q1, Q2, Dap, iX, iX_plus1] = getSegmentGoalParameters(X, x, inode);
                    fprintf('Moving to next segment, %d : (%.2f, %.2f), %d : (%.2f,%.2f)\n', inode, iX(1), iX(2), inode+1, iX_plus1(1), iX_plus1(2) );
                end
            end
        end
    end
    fprintf(', Goal : %.2f, %.2f\n',G(1), G(2));
end
