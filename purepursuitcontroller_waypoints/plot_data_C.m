function [] = plot_data_C(out,wayp,v_0,LD)
% function [] = plot_data_C(time,out.SAV_pos,out.SAV_angle,out.SAV_vel,out.SAV_rate,path_input)
maxtime=0.6*size(out.SAV_pos,1);
 time=[0.6:0.6:maxtime]';
figure(1); % path
plot(out.SAV_pos(:,1),out.SAV_pos(:,2),'r','LineWidth',1.5); 
hold on;
grid on;
plot(out.SAV_pos(:,3),out.SAV_pos(:,4),'b','LineWidth',1.5);
plot(out.SAV_pos(:,5),out.SAV_pos(:,6),'c','LineWidth',1.5); 
plot(out.SAV_pos(:,7),out.SAV_pos(:,8),'k','LineWidth',1.5);
plot(wayp(1,1),wayp(1,2),'g*','linewidth',2');
plot(out.SAV_pos(1,1),out.SAV_pos(1,2),'r*','LineWidth',2); 
plot(out.SAV_pos(1,3),out.SAV_pos(1,4),'b*','LineWidth',2);
plot(out.SAV_pos(1,5),out.SAV_pos(1,6),'c*','LineWidth',2);
plot(out.SAV_pos(1,7),out.SAV_pos(1,8),'k*','LineWidth',2);                         
szwp=size(wayp,1);

figure(1)                          %plotting waypoints(1)(13)
for i=1:szwp 
plot(wayp(i,1), wayp(i,2),  'g*','linewidth',1');
hold on
end
figure(1)
plot(wayp(1,1), wayp(1,2),  'r*','linewidth',2');

text(wayp(end,1)+68, wayp(end,2)+65, {'way points', num2str(wayp)},'fontsize',20,'EdgeColor', 'k');
text(wayp(end,1)+68, wayp(end,2)+20, {'Look-ahead R in m', num2str(LD)},'fontsize',20,'EdgeColor', 'k');
xlabel('x-position [m]','Fontsize',25);
ylabel('y-position [m]','Fontsize',25);
legend ('steer axle','drive axle','king-pin','semitrailer axle','way points','Location','southeast','Fontsize',20);
if v_0>0
title('Simulation of Single Articulated Vehicle along waypoints in Forward direction','Fontsize',25);
elseif v_0<0
title('Simulation of Single Articulated Vehicle along waypoints in Reverse direction','Fontsize',25);   
end
ax = gca;
% Set x and y font sizes.
ax.XAxis.FontSize = 24;
ax.YAxis.FontSize = 24;
axis equal
% %for saving as the fig and jpg
% fig=gcf;
% fig.WindowState='maximized';
% saveas(figure(1),[pwd '/images/path.fig']);
% saveas(figure(1),[pwd '/images/path.jpg']);
% 
figure(2); % yaw rates
plot(time,out.SAV_rate(:,1)*180/pi,'LineWidth',1.5); 
hold on;
grid on;
plot(time,out.SAV_rate(:,2)*180/pi,'LineWidth',1.5);
xlabel('time [s]','Fontsize',25);
ylabel('yaw rate [deg/s]','Fontsize',25);
legend ('tractor','semitrailer','Fontsize',25);
title('Yaw rate','Fontsize',25);
ax = gca;
% Set x and y font sizes.
ax.XAxis.FontSize = 24;
ax.YAxis.FontSize = 24;
% %for saving as the fig and jpg
% fig=gcf;
% fig.WindowState='maximized';
% saveas(figure(2),[pwd '/images/yawrate.fig']);
% saveas(figure(2),[pwd '/images/yawrate.jpg']);
% 
figure(3); % yaw angles 
plot(time,out.SAV_angle(:,1)*180/pi,'LineWidth',1.5); 
hold on;
grid on;
plot(time,out.SAV_angle(:,2)*180/pi,'LineWidth',1.5);
xlabel('time [s]','Fontsize',25);
ylabel('yaw angle [deg]','Fontsize',25);
legend ('tractor','semitrailer','Fontsize',25);
title('Yaw angle','Fontsize',25);
ax = gca;
% Set x and y font sizes.
ax.XAxis.FontSize = 24;
ax.YAxis.FontSize = 24;
% %for saving as the fig and jpg
% fig=gcf;
% fig.WindowState='maximized';
% saveas(figure(3),[pwd '/images/yawangle.fig']);
% saveas(figure(3),[pwd '/images/yawangle.jpg']);


figure(4); % unit velocities in local coordinate system
plot(time,out.SAV_vel(:,1),'LineWidth',1.5);
hold on;
grid on;
plot(time,out.SAV_vel(:,2),'LineWidth',1.5);
xlabel('time [s]','Fontsize',25);
ylabel('longitudinal velocity [m/s]','Fontsize',25);
legend ('tractor','semitrailer','Fontsize',25);
title('Local longitudinal velocity','Fontsize',25);
ax = gca;
% Set x and y font sizes.
ax.XAxis.FontSize = 24;
ax.YAxis.FontSize = 24;
% %for saving as the fig and jpg
% fig=gcf;
% fig.WindowState='maximized';
% saveas(figure(4),[pwd '/images/longvel.fig']);
% saveas(figure(4),[pwd '/images/longvel.jpg']);

figure(5); % steering angles
plot(time,out.SAV_angle(:,4)*180/pi,'LineWidth',1.5); 
hold on;
grid on;
plot(time,out.SAV_angle(:,5)*180/pi,'LineWidth',1.5);
xlabel('time [s]','Fontsize',25);
ylabel('steering angle [deg]','Fontsize',25);
legend ('actual','virtual','Fontsize',25);
title('Steering angle','Fontsize',25);
ax = gca;
%Set x and y font sizes.
ax.XAxis.FontSize = 24;
ax.YAxis.FontSize = 24;
% %for saving as the fig and jpg
% fig=gcf;
% fig.WindowState='maximized';
% saveas(figure(5),[pwd '/images/steeringangle.fig']);
% saveas(figure(5),[pwd '/images/steeringangle.jpg']);

figure(6); % steering angle rate
plot(time,out.steering_rate(:)*180/pi,'LineWidth',1.5); 
hold on;
grid on;
plot(time,out.steering_rate(:,1)*180/pi,'LineWidth',1.5);
xlabel('time [s]','Fontsize',25);
ylabel('steering rate [deg/sec]','Fontsize',25);
legend ('actual','virtual','Fontsize',25);
title('steering rate','Fontsize',25);
ax = gca;
%Set x and y font sizes.
ax.XAxis.FontSize = 24;
ax.YAxis.FontSize = 24;
% %for saving as the fig and jpg
% fig=gcf;
% fig.WindowState='maximized';
% saveas(figure(6),[pwd '/images/steeringrate.fig']);
% saveas(figure(6),[pwd '/images/steeringrate.jpg']);

figure(7); % steering acceleration
plot(time,out.steering_acceleration(:)*180/pi,'LineWidth',1.5); 
hold on;
grid on;
plot(time,out.steering_acceleration(:,1)*180/pi,'LineWidth',1.5);
xlabel('time [s]','Fontsize',25);
ylabel('steering acceleration [deg/secsquare]','Fontsize',25);
legend ('actual','virtual','Fontsize',25);
title('Steering acceleration','Fontsize',25);
ax = gca;
%Set x and y font sizes.
ax.XAxis.FontSize = 24;
ax.YAxis.FontSize = 24;
% %for saving as the fig and jpg
% fig=gcf;
% fig.WindowState='maximized';
% saveas(figure(7),[pwd '/images/steeringacceleration.fig']);
% saveas(figure(7),[pwd '/images/steeringacceleration.jpg']);

figure(8); % articulation angle
plot(time,out.SAV_angle(:,3)*180/pi,'LineWidth',1.5);
hold on;
grid on;
xlabel('time [s]','Fontsize',25);
ylabel('Articulation angle [deg]','Fontsize',25);
legend('articulation angle','Fontsize',25);
title('Articulation angle','Fontsize',25);
ax = gca;
%Set x and y font sizes.
ax.XAxis.FontSize = 24;
ax.YAxis.FontSize = 24;
% %for saving as the fig and jpg
% fig=gcf;
% fig.WindowState='maximized';
% saveas(figure(8),[pwd '/images/articulationangle.fig']);
% saveas(figure(8),[pwd '/images/articulationangle.jpg']);

figure(9); % yaw rates
plot(time,out.IK_SAV_rate(:,2)*(-180/pi),'LineWidth',1.5); 
hold on;
grid on;
plot(time,out.SAV_rate(:,2)*(180/pi),'LineWidth',1.5);
xlabel('time [s]','Fontsize',25);
ylabel('yaw rate [deg/s]','Fontsize',25);
legend ('IK-theta_1dot','KM-theta_1dot','Fontsize',25);
title('Semitrailer Yaw rate - Forward','Fontsize',25);
ax = gca;
%Set x and y font sizes.
ax.XAxis.FontSize = 24;
ax.YAxis.FontSize = 24;
% %for saving as the fig and jpg
% fig=gcf;
% fig.WindowState='maximized';
% saveas(figure(9),[pwd '/images/yawrate.fig']);
% saveas(figure(9),[pwd '/images/yawrate.jpg']);




