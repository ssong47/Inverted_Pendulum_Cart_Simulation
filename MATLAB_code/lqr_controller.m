function [K, Nbar, r_new] = lqr_controller(X,x_desired,p)
% Computes the necessary LQR controller parameters such as gains (K),
% precompensator (Nbar), and reference signal (r) 
global Nbar_array K_array

%% Defining Linear System of Cart-Pole System
[A,B,C,D] = compute_cart_pole_linear_system(X,x_desired,p);

% Defining Q matrix(weight on performance)
Q = C'*C;
Q(1,1) = p.Q1; 
Q(2,2) = p.Q2; 
Q(3,3) = p.Q3;
Q(4,4) = p.Q4;

% Defining R matrix (weight on control effort)
R = p.R;

% Obtaining K matrix (Gains of LQR)
% K = lqr(A,B,Q,R);
K = [-31.6228, 113.3629, -240.1784, 33.5693];

% K_array = [K_array; K];
% Obtaining Precompensation (Nbar)
Nbar = -31.6228;
% Nbar is computed using the two command lines below. Nbar does not change,
% so the constant is explicitly defined to save computation time.
% sys_ss = ss(A,B,C,D);
% Nbar = rscale(sys_ss, K);

% Saturating motor voltage to 12V,-12V
r = x_desired;

input_voltage = r * Nbar - K * X; 


if p.saturate_motor == 1
   r_new = saturate_voltage(input_voltage, p, K, Nbar, X);
elseif p.saturate_motor == 0
   r_new = r; 
end




end