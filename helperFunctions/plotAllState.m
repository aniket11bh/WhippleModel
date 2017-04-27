function [ ] = plotAllState(Y)
%PLOTALLSTATE plots
% phi, delta, psi, and trajectory of the bicycle

    t = Y(:,1);
    
    figure
    plot(t, (180/pi)*Y(:,2:3) );
    title('Bicycle lean and steer');
    xlabel('Time t (sec)');
    ylabel('angle (degrees)');
    legend('\phi','\delta');
       
    figure
    plot(t, (180/pi)*Y(:,6));
    title('Yaw angle');
    xlabel('Time t (sec)');
    ylabel('angle (degrees)');
    legend('Yaw angle of the bicycle');

    figure
    plot(Y(:,7), Y(:,8));
    title('Trajectory of the bicycle');
    xlabel('x (m)');
    ylabel('y (m)');
    % legend('y_1','y_2')
    
    figure
    plot(t, Y(:,10:11));
    title('Velocity of the bicycle');
    xlabel(' time in sec');
    ylabel('vel (m/s)');
    legend('x_dot', 'y_dot');

end

