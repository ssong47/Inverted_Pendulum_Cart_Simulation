function plot_output(t, x_desired, Xout, Uout, t_settle)


figure(2)
subplot(5,1,1);
plot(t,Xout(:,1));
xlabel('time (sec)');
ylabel('cart position (m)');
ylim([-0.1 1]);
yline(0,'--');
xlim([0 t(end)]);
yline(x_desired,'-.');
xline(t_settle(1),'-.');


subplot(5,1,2);
plot(t,Xout(:,2) * 180/pi);
xlabel('time (sec)');
ylabel('angle (deg)');
ylim([-20 20]);
yline(0,'--');
xlim([0 t(end)]);
xline(t_settle(2),'-.');


subplot(5,1,3);
plot(t,Xout(:,3));
xlabel('time (sec)');
ylabel('cart speed (m/s)');
ylim([-0.4 0.4]);
yline(0,'--');
xlim([0 t(end)]);
xline(t_settle(3),'-.');
yticklabels({'-0.4', '0', '0.4'});
yticks([-0.4 0, 0.4])

subplot(5,1,4);
plot(t,Xout(:,4)* 180/pi);
xlabel('time (sec)');
ylabel('angular speed (deg/s)');
ylim([-100 100]);
yline(0,'--');
xlim([0 t(end)]);
xline(t_settle(4),'-.');


subplot(5,1,5);
plot(t,Uout);
xlabel('time (sec)');
ylabel('Motor Voltage (V)');
ylim([-24 24]);
yticklabels({'-24','-12', '0', '12','24'});
yticks([-24, -12, 0, 12, 24])
yline(0,'--');
yline(12,'r--');
yline(-12,'r--');
xlim([0 t(end)]);



width = 500;
height = 900;
set(gcf, 'Position',  [100, 100, width , height]) % [d_left d_bottom width height]

