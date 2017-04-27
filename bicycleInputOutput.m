function [ Yt ] = bicycleInputOutput( v, B2, C, D, u, Yt, ti, dt )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    global w;
    global invM;
    persistent X;
    
    if isempty(X)
        X = Yt(2:5)';
    end
    
    % obtain the new dynamics for diff. vel
    A = getStateTransitionMatrix(invM, v, B2);
    
    B = B2(:,2);
    sys_bic_cl  = ss( A, B, C, D);
    N_bar = dcgain(sys_bic_cl);         

    % calculate the new states
    X = RK4( @bicycleStateDot, X, A, B, u, dt, ti );
       
     % // Calculation of other states // % 
    
    % yaw rate (rad/s) and yaw (rad)
    psi_dot  = v*tan(X(2))/w; 
    psi = Yt(6) + psi_dot*dt;
    
        % convert yaw in to -180 to 180 range
        psi = atan2(sin(psi), cos(psi));

     % Position calculation [m]
    x_dot = v*cos(psi);
    y_dot = v*sin(psi);
    x = Yt(7) + x_dot*dt;
    y = Yt(8) + y_dot*dt;
    
    Yt = [ti X(1)/N_bar(1) ,  X(2)/N_bar(2) ,  X(3)/N_bar(1),   X(4)/N_bar(1),  psi, x, y, psi_dot, x_dot, y_dot]; 
end