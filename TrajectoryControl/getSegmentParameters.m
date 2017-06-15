function [alpha1, alpha2, theta, beta, iX, Q1, Q2, Z] = getSegmentParameters(X, x, inode)
%GET_SEGMENT_PARAMETERS Returns parameters needed for relevant segement
%calculation
%  Inputs :
%       X : vector of position vector of trajectory nodal points
%       x : position vector of robot [x y]'
%   inode : ith node, start from 1

    persistent I;
    persistent I_plus1;
    persistent I_plus2;
    Z = 0;

    numNode = size(X,2); % Total no. of nodes
    delP = 0.01; %[m] step length after path ends

    if inode <= numNode -2
        I = X(:, inode); % position vector of ith node
        I_plus1 = X(:, inode+1); % position vector of (i+1)th node
        I_plus2 = X(:, inode+2); % position vector of (i+2)th node

    elseif inode == numNode -1
        % when path ends, or 2 nodes are only there
        I = X(:, inode);
        I_plus1 = X(:, inode+1);
        dir  = I_plus1 - I;
        I_plus2 = I_plus1 + dir; %(dir/norm(dir))*delP;
    else
        Z = -1;
    end

    % Calculate relevant parameters
    % i.e, Q1, Q2, alpha1, alpha2, theta
        % alpha1: Angle between vector Q1 and ith vector segment
        % alpha2: Angle between vector Q2 and ith vector segment
        % theta : Angle between current segment i, and next segment i+1
        % beta : theta/2

     Q1 = x - I;
     Q2 = I_plus1 - x;
     P  = I_plus1 - I;
     P_plus1 = I_plus2 - I_plus1;

     % All angles and in rad.
     alpha1 = angle(Q1, P);
     alpha2 = angle(Q2, P);
     theta = angle(P, P_plus1);
     beta = angle(-P, P_plus1)/2;

     iX = I; % position vector of ith node
end
