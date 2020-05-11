function [A,B,C,D] = compute_cart_pole_linear_system(X,r,p)

m = p.m;
g = p.g;
r_pulley = p.r_pulley;
M = p.M;
l = p.l;
Kv = p.Kv;
Kt = p.Kt;
Rw = p.Rw;
I_rotor = p.I_rotor;
N_rotor = p.N_rotor;


theta = X(2);
x_dot = X(3);

%% Linearization using taylor series expansion
A = [ 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          0,                                                                                                                    1, 0;
 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          0,                                                                                                                    0, 1;
 0,                                                                                                   ((- I_rotor*m*sin(theta)*N_rotor^2*r_pulley^2 + 2*m*cos(theta)*sin(theta))*(Kt*Kv*m*x_dot*cos(theta)*N_rotor^2 - Kt*m*r*cos(theta)*N_rotor + Rw*l*m*sin(theta) - Rw*g*m*cos(theta)*sin(theta)))/(Rw*(I_rotor*m*cos(theta)*N_rotor^2*r_pulley^2 + M + m - m*cos(theta)^2)^2) - (Rw*l*m*cos(theta) - Rw*g*m*cos(theta)^2 + Rw*g*m*sin(theta)^2 + Kt*N_rotor*m*r*sin(theta) - Kt*Kv*N_rotor^2*m*x_dot*sin(theta))/(Rw*(I_rotor*m*cos(theta)*N_rotor^2*r_pulley^2 + M + m - m*cos(theta)^2)),            -(Kt*Kv*N_rotor^2*m*cos(theta))/(Rw*(I_rotor*m*cos(theta)*N_rotor^2*r_pulley^2 + M + m - m*cos(theta)^2)), 0;
 0, (M*Rw*g*cos(theta) + Rw*g*m*cos(theta) - Rw*l*m*cos(theta)^2 + Rw*l*m*sin(theta)^2 + I_rotor*N_rotor^2*Rw*l*m*r_pulley^2*cos(theta))/(Rw*l*(I_rotor*m*cos(theta)*N_rotor^2*r_pulley^2 + M + m - m*cos(theta)^2)) - ((- I_rotor*m*sin(theta)*N_rotor^2*r_pulley^2 + 2*m*cos(theta)*sin(theta))*(M*Rw*g*sin(theta) + Rw*g*m*sin(theta) + Kt*M*N_rotor*r + Kt*N_rotor*m*r - Rw*l*m*cos(theta)*sin(theta) - Kt*Kv*M*N_rotor^2*x_dot - Kt*Kv*N_rotor^2*m*x_dot + I_rotor*N_rotor^2*Rw*l*m*r_pulley^2*sin(theta)))/(Rw*l*(I_rotor*m*cos(theta)*N_rotor^2*r_pulley^2 + M + m - m*cos(theta)^2)^2), -(Kt*Kv*N_rotor^2*m + Kt*Kv*M*N_rotor^2)/(Rw*l*(I_rotor*m*cos(theta)*N_rotor^2*r_pulley^2 + M + m - m*cos(theta)^2)), 0];
 


B = [
                                                                                                         0;
                                                                                                         0;
       (Kt*N_rotor*m*cos(theta))/(Rw*(I_rotor*m*cos(theta)*N_rotor^2*r_pulley^2 + M + m - m*cos(theta)^2));
 (Kt*M*N_rotor + Kt*N_rotor*m)/(Rw*l*(I_rotor*m*cos(theta)*N_rotor^2*r_pulley^2 + M + m - m*cos(theta)^2))];
 

%% Linearization using Small angle approximation 
A = [ 0,                                                                                                            0,                                                                                    1, 0;
 0,                                                                                                            0,                                                                                    0, 1;
 0,                                                  (Rw*g*m - Rw*l*m)/(Rw*(I_rotor*m*N_rotor^2*r_pulley^2 + M)),                       -(Kt*Kv*N_rotor^2*m)/(Rw*(I_rotor*m*N_rotor^2*r_pulley^2 + M)), 0;
 0, (I_rotor*Rw*l*m*N_rotor^2*r_pulley^2 + M*Rw*g + Rw*g*m - Rw*l*m)/(Rw*l*(I_rotor*m*N_rotor^2*r_pulley^2 + M)), -(Kt*Kv*N_rotor^2*m + Kt*Kv*M*N_rotor^2)/(Rw*l*(I_rotor*m*N_rotor^2*r_pulley^2 + M)), 0];
 
B = [ 0;
                                                                         0;
                  (Kt*N_rotor*m)/(Rw*(I_rotor*m*N_rotor^2*r_pulley^2 + M));
 (Kt*M*N_rotor + Kt*N_rotor*m)/(Rw*l*(I_rotor*m*N_rotor^2*r_pulley^2 + M))];



if p.flag_ctrl == 1 || p.flag_ctrl == 0
    C = [1 0 0 0];
elseif p.flag_ctrl == 2
    C = [1 0 0 0; 0 1 0 0];
end
D = 0;
 


end