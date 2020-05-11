function dxdt = compute_dyn_cart_pole(t,X, x_desired, p)
% computes rate of change of states in the dynamics of the cart pole system 
% given the states, desired states.
t

%% Defining global variables
global t_prev integral_term_1_prev integral_term_2_prev ov_counter


%% Defining Linear System of Cart-Pole System
[A,B,C,D] = compute_cart_pole_linear_system(X,x_desired,p);

 
%% Applying Controllers  
if p.flag_ctrl == 0         %LQR
    [K, Nbar, r_new] = lqr_controller(X,x_desired,p);
    Ac = A - B*K;
    Bc = Nbar * B;
    dxdt = Ac * (X) + Bc * r_new;
    
    
elseif p.flag_ctrl == 1     %PID
    u_pid = pid_controller(X,x_desired,t,p);
    dxdt = A * X + B * u_pid;
    
    
elseif p.flag_ctrl == 2     %MPC
    u_mpc = mpc_controller(X,x_desired,p);
    dxdt = A * X + B * u_mpc;
    
end


%% Non-linear System Dynamics (For Reference)
% alpha = Kt/Rw*N_rotor;
% De = [-cos(theta)   l; (M+m)     -m*l*cos(theta)];
% Ce = [0;    m*l*sin(theta)];
% Ge = [-g*sin(theta);    0];
% Be = [alpha;    0];
% 
% J_rotor = [I_rotor * r_pulley^2*N_rotor^2    0;
%             0                   0];
%  
% B_damp = [Kv*Kt/Rw*N_rotor^2    0;
%             0                   0];
% 
%         
% % Computing double derivative of q 
% ddq = (De + J_rotor) \ (Be * r - Ce - B_damp * dq - Ge);
% 
% dxdt = [dq;ddq];
% 



end