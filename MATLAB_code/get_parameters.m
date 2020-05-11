function p = get_parameters()
% Defines the parameters of the cart-pole system

%% cart & pole specs
p.M = 0.5; % mass of cart (kg)
p.m = 0.2; % mass of end of pole (kg)
p.g = 9.81; % gravity constant (m/s^2) 
p.r_pulley = 0.006; % radius of pulley (m) 
p.l = 0.3; % length of pole (m)

%% motor specs
p.Kv = 0.0186; % speed constant of motor (rad/(s*V))
p.Kt = 0.0135; % torque constant of motor (Nm/A)
p.Rw = 1.3; % internal winding resistance (ohm)
p.I_rotor = 0.000007; % inertia of rotor (kg*m^2)
p.N_rotor = 26.9; % gear ratio 

%% controller specs
% LQR
p.Q1 = 1000;
p.Q2 = 4000;
p.Q3 = 0; 
p.Q4 = 0;
p.R = 1;


% PID 
p.Kp1 = -50;
p.Ki1 = 0;
p.Kd1 = -400;

p.Kp2 = 150;
p.Ki2 = 0;
p.Kd2 = 100;




%% Motor Saturation Specs
p.voltage_soft_limit = 12;
p.ov_counter_limit = 200;
p.ov_absolute_limit = 36;

