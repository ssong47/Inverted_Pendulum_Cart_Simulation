%% Check controllability and observability of the system at equilibrium

X = [0;0;0;0]; % states at equilibrium
U = 0; % effort at equilibrium
r = 1; % desired x_position 
p = get_parameters();
p.flag_ctrl = 1;


[A,B,C,D] = compute_cart_pole_linear_system(X,r,p); % Obtain Linear System
sys = ss(A,B,C,D)


Co = ctrb(A,B); % compute controllability matrix

unco = length(X) - rank(Co); % number of uncontrollable states

if unco == 0
    disp('The System is controllable at equilibrium');
else
    disp('The System is NOT controllable at equilibrium');
end


Ob = obsv(A,C);

unob = length(A) - rank(Ob);

if unob == 0
   disp('The System is observable at equilibrium');
else
    disp('The System is NOT observable at equilibrium');
end


