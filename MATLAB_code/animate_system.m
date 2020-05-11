function animate_system(tin, Xin, traj_din, uin, flag_movie)

% For animation
N_animate = 30;


if flag_movie == 1
    name = ['test.mp4'];
    vidfile = VideoWriter(name,'MPEG-4');
    open(vidfile);
end

[t,X] = even_sample(tin,Xin,N_animate);
[t,traj_d] = even_sample(tin,traj_din,N_animate);
[t,u] = even_sample(tin,uin,N_animate);


% animate
figure('Position',[200 100 1000 600]);

nt = length(t);
err = zeros(nt,2);
traj = zeros(nt,2);



l = 0.3;
lwidth = 3;

for ii = 1:nt
    
    %% The main animation
    subplot(2,2,1)
    hold on;grid on;box on;
    
    % plot pendulum
    plotPendulum(X(ii,:), traj_d(ii,1));

    
    
    %% The tracking error
    subplot(2,2,2)
    hold on;grid on;box on;

    err(ii,:) = traj_d(ii,:) - X(ii,1:2);
    
    yyaxis left
    plot(t(1:ii),err(1:ii,1),'Color','#0072BD');
    
    yyaxis right
    plot(t(1:ii),err(1:ii,2) * 180/pi,'Color','#D95319');
    
    yline(0, 'Color', 'Black','LineWidth',1, 'LineStyle','--');

    
    legend('e_x','e_{theta}');
    title('Tracking Error');
    yyaxis left 
    xlabel('Time [s]');
    ylabel('Error [m]');
    ylim([-1.5 1.5]);
    
    yyaxis right 
    ylabel('Theta [deg]');
    ylim([-200 200]);
    
    
    
    
    %% The control
    subplot(2,2,3)
    hold on;grid on;box on;
    
    plot(t(1:ii),u(1:ii),'Color','#0072BD')
    yline(0,'--');
    yline(12,'r--');
    yline(-12,'r--')
    title('Control Input')
    xlabel('Time [s]')
    ylabel('Motor Voltage [V]')
    ylim([-20 20]);
    
    %% The position

    subplot(2,2,4)
    hold on;grid on;box on;
    
    yyaxis left    
    a = plot(t(1:ii),X(1:ii,1),'Color','#0072BD');
    yline(traj_d(ii,1),'Color','#0072BD','LineStyle','--');
    
    yyaxis right 
    b = plot(t(1:ii),X(1:ii,2) * 180/pi,'Color','#D95319');
    yline(traj_d(ii,2),'Color','#D95319','LineStyle','--');
    
    
    legend([a(1) b(1)],'X','theta');
    title('Cart Position and Angle')
    
    yyaxis left
    xlabel('Time [s]')
    ylabel('Cart Position [m]')
    ylim([-0.5 2]);
    
    yyaxis right
    ylabel('Pendulum Angle [deg]')
    ylim([-200 200]);
    
    % animate
    pause(0.0001)
    
    % record video
    if flag_movie
        writeVideo(vidfile, getframe(gcf));
    end
    
    if ii < nt
        clf
    end
    
end

if flag_movie
    close(vidfile);
end

end


function plotPendulum(X, traj_d)

l = 0.3;

pos = X(1);
theta = X(2);
lwidth = 3;
yline(0,'Color','black', 'LineWidth', lwidth);


% Mass at the end
circle_center = [pos-l * sin(theta) , l * cos(theta)];
radii = 0.03;

% Cart
rect_w = 0.2; 
rect_h = 0.1;
rect_x = pos-rect_w/2;
rect_y = 0;


rectangle_position = [rect_x rect_y rect_w rect_h];
rectangle('Position',rectangle_position, 'EdgeColor', '#114AA1', 'FaceColor',[0 0.4470 0.7410], 'LineWidth',lwidth);


% Link 
% link_start = [pos, rect_y + rect_h/2];
% % disp(link_start)
% link_end = circle_center;
link_x = [pos, circle_center(1)];
link_y = [rect_y+rect_h, circle_center(2)];
plot(link_x, link_y, 'Color', '#303030', 'LineWidth', lwidth);
filledCircle(circle_center, radii, 1000,[0.8500 0.3250 0.0980]); 
viscircles(circle_center, radii, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', lwidth);


% plot desired trajectory
xline(traj_d(1),'Color','black', 'LineWidth', 1.5, 'LineStyle', '--');

axis equal
% xlim([-0.3 1])
ylim([-0.1 0.5]);
xlabel('x [m]','fontsize',12)
ylabel('y [m]','fontsize',12)


end







