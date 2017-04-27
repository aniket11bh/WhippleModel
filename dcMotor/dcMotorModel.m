dcMotorParams;
A_dc_motor = [0   1              0
              0  -b_dcM/J_dcM  K_dcM/J_dcM
              0  -K_dcM/L_dcM -R_dcM/L_dcM];
          
B_dc_motor = [ 0
               0
              1/L_dcM];
          
C_dc_motor = [1  0  0];

D_dc_motor = 0;

dc_motor_states = {'theta' 'theta_dot' 'i'};
dc_motor_inputs = {'V'};
dc_motor_outputs = {'theta'};

motor_ss = ss(A_dc_motor, B_dc_motor, C_dc_motor, D_dc_motor, ...
              'statename', dc_motor_states, ...
              'inputname', dc_motor_inputs, ...
              'outputname', dc_motor_outputs);

% % Adding integral state feedback % %

% Augmented state space %
Aa_dc_motor = [ A_dc_motor [0 0 0]' ; 
               -C_dc_motor 0];
Ba_dc_motor = [B_dc_motor
                 0];
Br_dc_motor = [0
               0
               0
               1];
Ca_dc_motor = [C_dc_motor 0];

Da_dc_motor = D_dc_motor;

% Pole locations for augmented system%
pa_dc_motor = [-110+100i -110-100i -200 -300];

Ka = place(Aa_dc_motor, Ba_dc_motor, pa_dc_motor);

t = 0:0.001:0.1;


sys_cl = ss( (Aa_dc_motor - Ba_dc_motor*Ka), Br_dc_motor, Ca_dc_motor, Da_dc_motor);
step(sys_cl, t);