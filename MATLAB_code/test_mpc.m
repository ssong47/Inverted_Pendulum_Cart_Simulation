

m = 0.2;
g = 9.81;
r_pulley = 0.006;
M = 0.5;
l = 0.3;
Kv = 0.0186;
Kt = 0.0135;
Rw = 1.3;
I_rotor = 0.000007;
N_rotor = 26.9;
p = get_parameters();
r = [1;0];
ts = 0.035;

t = [0:ts:20];
N = length(t);
y = zeros(N,2); 
u = zeros(N,1); 

% ts = 0.035; % sampling time 
prediction_horizon = 30; % prediction horizon
control_horizon = 2; % control horizon 
X= [0,0,0,0];


for i = 1:N
    mpcverbosity('off');
    
    [A,B,C,D] = compute_cart_pole_linear_system(X,r(1),p);
    plant = ss(A,B,C,D);
    
    mpc_obj = mpc(plant, ts, prediction_horizon,control_horizon,[]);
    x = mpcstate(mpc_obj);
%     mpc_obj.mv.Min = -12; % measured variable (i.e., motor voltage)
%     mpc_obj.mv.Max = 12; 
%     mpc_obj.Model.Plant.InputName = 'Motor_Voltage';
%     mpc_obj.Model.Plant.InputUnit = 'Voltage';
%     mpc_obj.Model.Nominal.X = [0;0;0;0];
%     mpc_obj.Model.Nominal.Y = [0;0];  
%     mpc_obj.Model.Plant.OutputName = {'x'; 'theta'};
%     mpc_obj.Model.Plant.OutputUnit = {'m'; 'rad'};
% 
%     mpc_obj.OutputVariables(1).MinECR = 1;
%     mpc_obj.OutputVariables(1).MaxECR = 1;
% 
%     mpc_obj.OutputVariables(2).Min = -0.05;
%     mpc_obj.OutputVariables(2).Max = 0.05;
%     mpc_obj.OutputVariables(2).MinECR = 1;
%     mpc_obj.OutputVariables(2).MaxECR = 1;
% 
%     mpc_obj.Weights.OutputVariables = [1 1];
    x = mpcstate(mpc_obj);
    X = x.Plant
    y(i,:) = C*x.Plant;
    u(i) = mpcmove(mpc_obj,x,y(i,:),r);
    
end

y