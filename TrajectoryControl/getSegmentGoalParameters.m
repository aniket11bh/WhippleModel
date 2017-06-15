function [eps, Q1, Q2, Dap, iX, iX_plus1] = getSegmentGoalParameters(X, x, inode)
%GET_SEGMENT_GOAL_PARAMETERS Returns parameters needed for relevant segement
%calculation
%  Inputs :
%       X : vector of position vector of trajectory nodal points
%       x : position vector of robot [x y]'
%   inode : ith node, start from 1

%     persistent I;
%     persistent I_plus1;

    numNode = size(X,2); % Total no. of nodes
    delP = 100; %[m] step length after path ends

    if inode <= numNode -1
        I = X(:, inode); % position vector of ith node
        I_plus1 = X(:, inode+1); % position vector of (i+1)th node

    else
        % when path ends, or last node
%         if isempty(I)
%             I = X(:, inode-1); % position vector of ith node
%             I_plus1 = X(:, inode); % position vector of (i+1)th node
%         end
        I = X(:, inode-1); % position vector of ith node
        I_plus1 = X(:, inode); % position vector of (i+1)th node
        dir  = I_plus1 - I;
        I = I_plus1;
        I_plus1 = I_plus1 + (dir/norm(dir))*delP;
    end

    Q1 = x - I;
    Q2 = I_plus1 - x;
    P  = I_plus1 - I;

    % All angles and in rad.
    alpha1 = angle(Q1, P);
    eps = -norm(Q1)*sin(alpha1);

%     L = [I(1) + norm(Q1)*I(1)*cos(alpha1)/norm(I)
%            I(2) + norm(Q1)*I(2)*cos(alpha1)/norm(I)];
    L = I + [cos(alpha1) -sin(alpha1) ;sin(alpha1)  cos(alpha1)]*Q1*cos(alpha1);

    Dap = -norm(Q1);
%     Dap = norm(L - iX);
%     Dap = norm(iX - iX_plus1);
    iX = I;
    iX_plus1 = I_plus1;

end
