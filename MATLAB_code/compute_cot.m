function cot = compute_cot(avg_power, x_desired, t_settle,p)

speed = x_desired/t_settle(1); 
cot = avg_power/(p.g * (p.m + p.M) * speed);

end