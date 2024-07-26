%This script can be used to plot the results of the firs numerica examples
%of the abstract submitted to Scitech.
%Goal: Move the base of the spacecraft to a desired reference while
%avoinding an obstacle (NOTE:the robot is in a constant configuration)


clc,close all,clear all

%load the matlab results
%xx = robot state (position and velocity) [x,y,psi,qr1,qr2,qr3,dx,dy,dpsi,dqr1,dqr2,dqr3]
%ref = reference point, t = time, u_cl = base command input
load('data_sim_base_new.mat') 


close all
print_system_config %print the animation to visualize the motion of the system


figure(2)
subplot(2,1,1) %X-Y base
plot(t,xx(1,:),'r',t,xx(2,:),'b','LineWidth',3)
hold on
plot(t,ref(1,:),'*','LineWidth',0.5)
plot(t,ref(2,:),'o','LineWidth',0.5)
legend('x_c','y_c','x_{c,des}','y_{c,des}')
ylabel('X-Y [m]')
xlabel('Time [s]')
grid on
subplot(2,1,2) %Psi base
plot(t,rad2deg(xx(3,:)),t,rad2deg(ref(3,:)),'LineWidth',3)
xlabel('Time [s]')
ylabel('\Psi [deg]')
legend('\psi','\psi_{des}')
grid on


%plot control input
figure(2)
for i = 1:size(u_cl,2)
hold on
grid on
plot(t,u_cl(:,i),'LineWidth',3)
xlabel('Time [s]')
ylabel('Control Input')
end
legend('u_{c1}','u_{c2}','u_{c3}')