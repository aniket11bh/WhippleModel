function [alpha] = angle(v1, v2)
% ANGLE calculates angle of v2 from vector v1
% CCW is positive
% -PI <= alpha <= PI
    
    z = atan2(v2(2), v2(1)) - atan2(v1(2), v1(1));
    alpha = atan2(sin(z), cos(z));
    
end