% Get the trajectory
X = generateTrajectory();

% Initial position of robot
x_i = 0;
y_i = 0;
psi_i = 0;
inode = 1;
La = 0; % Look ahead distance
b = 1; % wheelbase : [1 m]


% Search for relevant segments %
    
    % TODO : get current position of robot and velocity vector
    x = [x_i y_i]';
    V = [v_x V_y]';
    
    [alpha1, alpha2, beta] = getSegmentParameters(X, x, inode);
    [segment, Case] = isRelevant(alpha1, alpha2, beta);
    
    while segment == -1
        % move to next segment
        inode = inode +1;
        
        [alpha1, alpha2, beta, I, Q1] = getSegmentParameters(X, x, inode);
        [segment, Case] = isRelevant(alpha1, alpha2, beta);
        
    end
    
    
    inode = segment;
        
    % Relevant segment
    % Calculate Lx, Ly, eps, Dap
        
    eps = -norm(Q1)*sin(alpha1);
    L = [I(1) + norm(Q1)*I(1)*cos(alpha1)/norm(I)
            I(2) + norm(Q1)*I(2)*cos(alpha1)/norm(I)];
         
    % Dap for diff. cases it's case 3 or 4
    if Case == 1 || Case == 2
        Dap = norm(L - I);
    elseif Case == 3
        Dap = norm(I);
    elseif Case == 4
        Dap = -norm(Q1);
    end 
        
    G = findGoal(inode, eps, Dap, L, La);
        
    % eta : angle between G vector and current velocity vector
    eta = angle(G - x, V);
    k = 2*sin(eta)/La;
     
    % Steering angle req. in rad
    delta = atan(2*b*sin(eta)/La);