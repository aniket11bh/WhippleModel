function [ f4 ] = transientResponse( T, tfin, nsample )
%TRANSIENTRESPONSE Summary of this function goes here
%   Detailed explanation goes here
    %
% Transient responses of the system

    [T30, samples] = usample(T, nsample);
    time = 0:tfin/500:tfin;
    nstep = size(time,2);
    ref1(1:nstep) = 1.0; ref2(1:nstep) = 0.0;
    ref = [ref1' ref2'];
    f4 = figure(4);
    subplot(2,2,1)
    hold off
    for i = 1:nsample
        [y,t] = lsim(T30(1:2,1:2,i),ref,time);
        plot(t,y(:,1)*180/pi,'r-')
        hold on
        plot(t, ref1*180/pi,':');
    end 
    grid
    title('From \phi_{ref} to \phi')
    xlabel('Time (secs)')
    ylabel('\phi lean angle (degrees)')
    figure(4)
    subplot(2,2,3)
    hold off
    for i = 1:nsample
        [y,t] = lsim(T30(1:2,1:2,i),ref,time);
        plot(t,y(:,2)*180/pi,'b-')
        hold on
        plot(t, ref2*180/pi,':');
    end 
    grid
    title('From \phi_{ref} to \delta')
    xlabel('Time (secs)')
    ylabel('\delta steer angle (degrees)')
    %
    time = 0:tfin/500:tfin;
    nstep = size(time,2);
    ref1(1:nstep) = 0.0; ref2(1:nstep) = 1.0;
    ref = [ref1' ref2'];
    figure(4)
    subplot(2,2,2)
    hold off
    for i = 1:nsample
        [y,t] = lsim(T30(1:2,1:2,i),ref,time);
        plot(t,y(:,1)*180/pi,'r-')
        hold on
        plot(t, ref1*180/pi,':');
    end 
    grid
    title('From \delta_{ref} to \phi')
    xlabel('Time (secs)')
    ylabel('\phi lean angle (degrees)')
    %
    figure(4)
    subplot(2,2,4)
    hold off
    for i = 1:nsample
        [y,t] = lsim(T30(1:2,1:2,i),ref,time);
        plot(t,y(:,2)*180/pi,'b-')
        hold on
        plot(t, ref2*180/pi,':');
    end 
    grid
    title('From \delta_{ref} to \delta')
    xlabel('Time (secs)')
    ylabel('\delta steer angle (degrees)')
    clear ref1;  clear ref2;

end

