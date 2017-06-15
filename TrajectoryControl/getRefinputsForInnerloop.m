function [ U ] = getRefinputsForInnerloop(X)
%GET_REF_INPUTS_FOR_INNERLOOP returns the reference velocites 
% and reference steering angles for a pre planned trajectory
% 
%   Inputs : X = trajectory points
%   output : U = [[v_ref; delta_ref], [], .. [] ]

    global bic;
    
    U = ones(size(X));
    
    R = bic.w;
    delta = atan(bic.w/R);
    U(1,:) = U(1,:)*1;
    U(2,:) =  U(2,:)*delta;
    
end

