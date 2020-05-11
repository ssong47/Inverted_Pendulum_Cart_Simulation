function avg_power = compute_avg_power(Uout, x_dot, p)

N = length(Uout);
current = zeros(N,1);
power = zeros(N,1);

for i = 1:N
    omega = x_dot(i) / p.r_pulley;
    current(i) = (Uout(i) - p.Kv * omega)/p.Rw;
    power(i) = current(i) * Uout(i);
end

avg_power = mean(power);

end
