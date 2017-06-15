function  [inode, eps, Q1, Q2, Dap, L, Z] = propagateNode( X, x, inode )
%PROPAGATE_NODE searches for the relevant node and find the 
% relevant parameters

    segmentID = -1; % segmentID calculated here will the relevant segment

    while segmentID == -1

        [alpha1, alpha2, theta, beta, iX, Q1, Q2, Z] = getSegmentParameters(X, x, inode);

        if Z == -1
            % path ends
            Case = 1;
            break;
        end
        
        [segmentID, Case] = isRelevant(alpha1, alpha2, theta, beta, inode);

        if segmentID == -1
            % move to next segment
            inode = inode +1;
        end

    end

    % Relevant segment
    % Calculate Lx, Ly, eps, Dap

    eps = -norm(Q1)*sin(alpha1);
    %L = [iX(1) + norm(Q1)*iX(1)*cos(alpha1)/norm(iX)
    %      iX(2) + norm(Q1)*iX(2)*cos(alpha1)/norm(iX)];
    L = iX + [cos(alpha1) -sin(alpha1) ;sin(alpha1)  cos(alpha1)]*Q1*cos(alpha1);

    % Dap for diff. cases it's case 3 or 4, it is vehicles distance along relevant path segment
    if Case == 1 || Case == 2
        % Inside or Outside of the turn
        Dap = norm(L - iX);

    elseif Case == 3
        % Deadzone, length of ith path segment
        iX_plus1 = X(:, inode+1);
        Dap = norm(iX - iX_plus1);

    elseif Case == 4
        % In front of the first node, assigned a negative value
        Dap = -norm(Q1);
    end

end
