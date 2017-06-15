clear all
close all
clc

% Bicycle model

% Load Bicycle parameters
addpath('params');
addpath('helperFunctions');

bicycleParam;
dcMotorParam;

% Balancing mecha. Spring const. and damping const.
bic_states = {'phi' 'delta' 'phi_dot' 'delta_dot'};
bic_inputs = {'leanT' 'steerT'};
bic_outputs = {'phi','steer'};

for v = 0.5:0.5:6.95

    A = getStateTransitionMatrix(v, bic.B, 3);
    B = bic.B; 
    C = bic.C;
    D = bic.D;


    sys_bic = ss(A, B, C, D, ...
                         'statename', bic_states,...
                         'inputname', bic_inputs,...
                         'outputname', bic_outputs);

    N_bar = dcgain(sys_bic);   

    sys_bic_norm  = ss(A, B/N_bar, C, D, ...
                                    'statename', bic_states,...
                                    'inputname', bic_inputs,...
                                    'outputname', bic_outputs);
    for del = 0:pi/18:pi*0.44
        x0 = [0 0 0 0]; 

        t = 0:0.01:5;

        l = length(t);
        l_hf = floor(l/2);
        
        % Reference var.
        r  = [0*ones(1,l)
                0*ones(1,l_hf) del*ones(1,l - l_hf)];

        [y, t, x] = lsim(sys_bic_norm, r, t, x0);
        yyaxis left
        hold on
        plot(t, y(:,1)*180/pi,'-');
%         hold on
        plot(t, r(1,:)*180/pi,':');
        ylabel('Lean angle (degrees)')

        yyaxis right
        plot(t, y(:,2)*180/pi,'-');
        plot(t, r(2,:)*180/pi, ':');
        plot(t, 80*ones(1,l),':');
        ylabel('Steer angle (degrees)');

        hold off;
        title('Step Response of steer and lean angles')
        xlabel('time (sec)')
%         pause
    end
end
