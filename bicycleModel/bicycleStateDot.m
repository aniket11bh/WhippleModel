function [ X_dot ] = bicycleStateDot(t, X, A, B, U )
%BICYCLE_STATE_DOT calculates x_dot

X_dot = A*X + B*U;

end

