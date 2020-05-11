function u = mpc_controller(X,x_desired,p)
    mpcverbosity('off');
    r = [x_desired; 0];
    
    [A,B,C,D] = compute_cart_pole_linear_system(X,x_desired,p);
    plant = ss(A,B,C,D);
    
    ts = 0.005; % sampling time 
    prediction_horizon = 100; % prediction horizon
    control_horizon = 2; % control horizon 
    
    mpc_obj = mpc(plant, ts,prediction_horizon,control_horizon,[]);
    
    mpc_obj.mv.Min = -12; % measured variable (i.e., motor voltage)
    mpc_obj.mv.Max = 12; 
    mpc_obj.Model.Plant.InputName = 'Motor_Voltage';
    mpc_obj.Model.Plant.InputUnit = 'Voltage';
    mpc_obj.Model.Nominal.X = [0;0;0;0];
    mpc_obj.Model.Nominal.Y = [0;0];  
    mpc_obj.Model.Plant.OutputName = {'x'; 'theta'};
    mpc_obj.Model.Plant.OutputUnit = {'m'; 'rad'};

    mpc_obj.OutputVariables(1).MinECR = 1;
    mpc_obj.OutputVariables(1).MaxECR = 1;

    mpc_obj.OutputVariables(2).Min = -0.05;
    mpc_obj.OutputVariables(2).Max = 0.05;
    mpc_obj.OutputVariables(2).MinECR = 1;
    mpc_obj.OutputVariables(2).MaxECR = 1;
  
    mpc_obj.Weights.OutputVariables = [1 1];
    
    x = mpcstate(mpc_obj);
%     
%     plant_output = X(1:2)';
%     u = mpcmove(mpc_obj, initial_state, plant_output, r); 
%     
    
    t = [0:ts:40];
    N = length(t);
    y = zeros(N,4); 
    u = zeros(N,1); 
    
    for i = 1:N
        % simulated plant and predictive model are identical
        y(i,:) = x.Plant;
        u(i) = mpcmove(mpc_obj,x,y(i,1:2),r);
    end
    
    plot(t, y(:,1)) 


end