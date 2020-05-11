% Cart Pole Simulation
% Author: Seung Yun (Leo) Song
% Last modified: 05/02/2020


clear all
close all
clc

% --- global variables ---
global t_prev integral_term_1_prev integral_term_2_prev ov_counter Nbar_array K_array

t_prev = 0;
ov_counter = 0;
integral_term_1_prev = 0;
integral_term_2_prev = 0;


% --- parameters ---
% Get the parameters about the robot and simulation settings
p = get_parameters();


% --- flag control ---
% Choose desired type of controller for the system 
% 0 - LQR controller mode
% 1 - PID controller mode
% 2 - MPC controller mode 
p.flag_ctrl = 2;

% --- motor saturation ---
% 1 - saturate motor to +-12V. 0 - Do not saturate motor
p.saturate_motor = 1; 

% --- initialization ---
% Initialize the system states, time (sec), desired cart position (m) 
% states: cart position, pole angle, cart speed, pole angular speed)
ic = [0, 0, 0, 0];

tstart = 0;
tfinal = 25;
x_desired = 0.5;

% --- definition of simulation parameters -- 
tout = tstart;
Xout = ic;


% --- starting simulation using ode45 --- 
if p.flag_ctrl == 0 || p.flag_ctrl == 1
    tic;
    [t,X] = ode45(@(t,X)compute_dyn_cart_pole(t, X, x_desired, p),[tstart, tfinal], Xout); 
    toc;
    nt = length(t);
    tout = [tout;t(2:nt)];
    Xout = [Xout;X(2:nt,:)];

elseif p.flag_ctrl == 2
    [m, g, r_pulley, M, l, Kv, Kt, Rw, I_rotor, N_rotor] = set_parameters(p);
    r = 0.5;
    X = [0;0;0;0];
    [A,B,C,D] = compute_cart_pole_linear_system(X,r,p);
    mpcverbosity('off');
    load('mpc_v2.mat');
    ref = [x_desired; 0];
    tic;
    simOut = sim('inverted_pend_2.slx',tfinal);
    toc;
    tout = simOut.tout;
    Xout = zeros(length(tout),2);
    for i_state = 1:4
        Xout(:,i_state) = simOut.Xout(:,i_state);
%         Xout(:,i_state) = simOut.Xout(:,i_state);

    end
    
    Uout = simOut.Uout;
    pad = zeros(length(tout) - length(Uout),1);
    Uout = [Uout; pad];
    
end
%%

desired_trajectory = [x_desired * ones(length(tout),1),...
zeros(length(tout),1)];     %[desired cart position, desired pole angle]


x = Xout(:,1);
theta = Xout(:,2);
x_dot = Xout(:,3);
theta_dot = Xout(:,4);

theta_desired = 0;
x_dot_desired = 0;
theta_dot_desired = 0;
    
% --- obtaining control effort ---
if p.flag_ctrl == 0 || p.flag_ctrl == 1
    Uout = find_control_effort(tout, Xout, desired_trajectory, p);
end



% --- animation ---
flagmovie = 1;
animate_system(tout,Xout,desired_trajectory, Uout,flagmovie);



% --- finding disk margin --- 
% disk_margin_results = find_disk_margin(p);



% --- analyzing response ---
t_settle = zeros(4,1);
t_rise = zeros(4,1);
overshoot = zeros(4,1);
e_steady_state = zeros(4,1);

[t_settle(1), t_rise(1), overshoot(1), e_steady_state(1)] = analyze_response...
    (tout, x, x_desired, 1);
[t_settle(2), t_rise(2), overshoot(2), e_steady_state(2)] = analyze_response...
    (tout, theta, theta_desired, 0);
[t_settle(3), t_rise(3), overshoot(3), e_steady_state(3)] = analyze_response...
    (tout, x_dot, x_dot_desired, 0);
[t_settle(4), t_rise(4), overshoot(4), e_steady_state(4)] = analyze_response...
    (tout, theta_dot, theta_dot_desired, 0);

avg_power = compute_avg_power(Uout, x_dot, p);

cot = zeros(4,1);
cot(1) = compute_cot(avg_power, x_desired, t_settle(1), p);
cot(2) = compute_cot(avg_power, x_desired, t_settle(2), p);
cot(3) = compute_cot(avg_power, x_desired, t_settle(3), p);
cot(4) = compute_cot(avg_power, x_desired, t_settle(4), p);

% --- plotting response and control effort --- 
plot_output(tout,x_desired, Xout, Uout, t_settle);





