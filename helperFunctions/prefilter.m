function [ r ] = prefilter( r, dcgain )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    n = length(r);
    for i =1 : n
        r(:,i) = inv(dcgain)*r(:,i);
    end
end

