function disk_margin_results = find_disk_margin(p)
% Analyze robustness using disk margin for each controller

X = [0,0,0,0]; % Robustness analyzed at the point of equilibrium
r = 0; % no reference signal for robustness analysis

states = {'x' 'theta' 'x_dot' 'theta_dot'};
inputs = {'F'};
outputs = {'x'};

if p.flag_ctrl == 0
    [A,B,C,D] = compute_cart_pole_linear_system(X,r,p);

    plant_state_space = ss(A,B,C,D, 'statename',states,'inputname',inputs,...
        'outputname',outputs); % define plant in state space form
    
    plant_tf = tf(plant_state_space); % convert plant in transfer function form
    
    [K_gain, Nbar, r_new] = lqr_controller(X',r,p);
    
    uout = r_new * Nbar - K_gain * X; 
    
    controller = 1 / (1 + K_gain * plant_tf - plant_tf);      % compute Controller (C) that is in series with plant 
    
    disk_margin_results = diskmargin(plant_tf,controller);
    
    
elseif p.flag_ctrl == 1
%     disk_margin_results = 
    
    
elseif p.flag_ctrl == 2
%     disk_margin_results = 
    
    
    
end

end