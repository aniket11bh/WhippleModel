function  [inode, eps, Q1, Q2, Dap, L, Z] = propagateNode( X, x, inode )
%PROPAGATE_NODE Summary of this function goes here
%   Detailed explanation goes here

    segmentID = -1;
   
    while segmentID == -1
        
        [alpha1, alpha2, theta, beta, iX, Q1, Q2, Z] = getSegmentParameters(X, x, inode);
        [segmentID, Case] = isRelevant(alpha1, alpha2, theta, beta, inode);
        
        if segmentID == -1
            % move to next segment
            inode = inode +1;
        end
        
    end
        
    % Relevant segment
    % Calculate Lx, Ly, eps, Dap
        
    eps = -norm(Q1)*sin(alpha1);
    L = [iX(1) + norm(Q1)*iX(1)*cos(alpha1)/norm(iX)
            iX(2) + norm(Q1)*iX(2)*cos(alpha1)/norm(iX)];
         
    % Dap for diff. cases it's case 3 or 4
    if Case == 1 || Case == 2
        Dap = norm(L - iX);
    elseif Case == 3
        Dap = norm(iX);
    elseif Case == 4
        Dap = -norm(Q1);
    end 

end

