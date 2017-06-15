function [SegmentID, Case] = isRelevant(alpha1, alpha2, theta, beta, inode)
%IS_RELEVANT outputs whether given segment is relvant or not
% Inputs :
% alpha1: Angle between vector Q1 and ith vector segment
% alpha2: Angle between vector Q2 and ith vector segment
% beta : half of Angle between current segment i, and next segment i+1

% TODO : search for all possible relevants among all segments

% Cases :
    % 1 : Inside of turn
    % 2 : outside of turn
    % 3 : Dead zone
    % 4 : First node in front of vehicle

    % Relevancy condition %
    if abs(alpha1) <= pi/2
        if (alpha2 <0 && beta >0) || (alpha2 >0 && beta <0)
            if abs(alpha2) < abs(beta)
                % case 1, Segment Relevant
                fprintf('Inside of turn\n');
                SegmentID = inode;
                Case = 1;
            else
                % case 0, Go for next segment
                SegmentID = -1;
                fprintf('moving for next segment\n');
%                 pause;
                Case = 0;
            end
        else
            if abs(alpha2) <= pi/2
               % case 2, Segment Relevant
               fprintf('Outside of turn\n');
               SegmentID = inode;
               Case = 2;
            else
                if abs(alpha2) - abs(theta) - pi/2 < 0
                    % case 3, Segment Relevant
                    fprintf('Dead zone\n');
                   SegmentID = inode;
                   Case = 3;
                else
                    % case 0, Go for next segment
                    SegmentID = -1;
                    fprintf('moving for next segment\n');
%                     pause;
                    Case = 0;
                end
            end
        end

    else
        if inode == 1
            % case 4, Segment Relevant, Front node in front of the vehicle
            fprintf('Front of first node\n');
            SegmentID = inode;
            Case = 4;
        else
            % case 0, Go for next segment, when will this case occur ?
            SegmentID = -1;
            fprintf('moving for next segment, unknown case\n');
            pause;
            Case = 0;
        end
    end
end
