function [ ] = plotAllState(X, Y)
%PLOTALLSTATE plots
% phi, delta, psi, and trajectory of the bicycle

    t = Y(:,1);
    
    figure
    plot(t, (180/pi)*Y(:,2:3) );
    hold on;
    plot(t, (180/pi)*Y(:,12));
    hold off;
    title('Bicycle lean and steer');
    xlabel('Time t (sec)');
    ylabel('angle (degrees)');
    legend('\phi','\delta', '\delta_{ref}');
       
    figure
    plot(t, (180/pi)*Y(:,6));
    title('Yaw angle');
    xlabel('Time t (sec)');
    ylabel('angle (degrees)');
    legend('Yaw angle of the bicycle');

    figure
    plot(Y(:,7), Y(:,8));
    hold on
    plot(X(1,:), X(2,:));
    hold off
    title('Trajectory');
    xlabel('x (m)');
    ylabel('y (m)');
    legend('Trajectory followed by bicycle','Desired trajectory')
    
    figure
    plot(t, Y(:,10:11));
    title('Velocity of the bicycle');
    xlabel(' time in sec');
    ylabel('vel (m/s)');
    legend('x_{dot}', 'y_{dot}');

end

