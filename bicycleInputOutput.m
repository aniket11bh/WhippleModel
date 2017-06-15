function [ Yt ] = bicycleInputOutput( v, u, Yt, ti, dt )
%BICYCLE_INPUT_OUTPUT returns [ti, X, psi, x, y, psi_dot, x_dot, y_dot]
%   Inputs : 
%       v = velocity of the bicycle
%       u = reference input  : [phi_ref; delta_ref]
%       Yt = 
%       ti = timestamp
%       dt = timestep

    global bic;
    persistent X;
    
    if isempty(X)
        X = Yt(2:5)';
    end
    
    % obtain the new dynamics for different vel
    A = getStateTransitionMatrix(v, bic.B, 3);
    
    sys_bic_cl  = ss( A, bic.B, bic.C, bic.D);
    N_bar = dcgain(sys_bic_cl);         

    sys_bic_norm  = ss(A, bic.B/N_bar, bic.C, bic.D);
    
    A = sys_bic_norm.A;
    B = sys_bic_norm.B;
    C = sys_bic_norm.C;
    D = sys_bic_norm.D;
   
    % calculate the new states
    X = RK4( @bicycleStateDot, X, A, B, u, dt, ti );
       
     % // Calculation of other states // % 
    
    % yaw rate (rad/s) and yaw (rad)
    psi_dot  = v*tan(X(2))/bic.w; 
    psi = Yt(6) + psi_dot*dt;
    
    % convert yaw in to -180 to 180 range
    psi = atan2(sin(psi), cos(psi));

     % Position calculation [m]
    x_dot = v*cos(psi);
    y_dot = v*sin(psi);
    x = Yt(7) + x_dot*dt;
    y = Yt(8) + y_dot*dt;
    
    Yt = [ti X(1) ,  X(2) ,  X(3),   X(4),  psi, x, y, psi_dot, x_dot, y_dot, u(2)]; 
end