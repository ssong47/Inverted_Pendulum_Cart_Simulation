function u_pid_new = pid_controller(X, x_desired, t_cur, p)
% Apply PID controllers for controlling cart position to desired x_desired
% and pole angle to zero. 

% global t_prev integral_term_1_prev integral_term_2_prev ov_counter

x = X(1);
theta = X(2);
x_dot = X(3);
theta_dot = X(4);

Kp1 = p.Kp1;
Ki1 = p.Ki1;
Kd1 = p.Kd1;

Kp2 = p.Kp2;
Ki2 = p.Ki2;
Kd2 = p.Kd2;

% integral_term_1_new = (t_cur - t_prev) * (x_desired - x);
% integral_term_2_new = (t_cur- t_prev) * (-theta);

integral_term_1_new = 0;
integral_term_2_new = 0;

integral_term_1_prev = 0;
integral_term_2_prev = 0;

u_pid = Kp1 * (x_desired - x) + Kd1 * (-x_dot) + Ki1 * (integral_term_1_new + integral_term_1_prev);
u_pid = u_pid + Kp2 * (-theta) + Kd2 * (-theta_dot) + Ki2 * (integral_term_2_new + integral_term_2_prev);

% t_prev = t_cur;
% integral_term_1_prev = integral_term_1_new + integral_term_1_prev;
% integral_term_2_prev = integral_term_2_new + integral_term_2_prev;

if p.saturate_motor == 1
    u_pid_new = saturate_voltage(u_pid,p);
elseif p.saturate_motor == 0
    u_pid_new = u_pid;
end

 



end