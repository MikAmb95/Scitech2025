%This script can be used to plot the results of the second numerica examples
%of the abstract submitted to Scitech.
%Goal: Move the robotic arm to a desired reference while
%(NOTE:the base is in a floating configuration)

clc,close all,clear all

%load the matlab results
%xx = robot state (position and velocity)
%[x,y,psi,qr1,qr2,qr3,dx,dy,dpsi,dqr1,dqr2,dqr3] /xr is the same but for
%the plot
%ref = reference, t = time, u_cl = base command input
load('data_sim_robot_new.mat')

print_system_config %print the animation to visualize the motion of the system

figure(2)
hold on
grid on
plot(t,rad2deg(xr(4,:)),t,rad2deg(ref(1,:)),'LineWidth',3)
plot(t,rad2deg(xr(5,:)),t,rad2deg(ref(2,:)),'LineWidth',3)
plot(t,rad2deg(xr(6,:)),t,rad2deg(ref(3,:)),'LineWidth',3)
xlabel('Time [s]','FontSize',20)
legend('q_{r1}','q_{rd1}','q_{r2}','q_{rd2}','q_{r3}','q_{rd3}')
ylabel('Joint [rad]');


figure(3)
grid on
hold on
plot(t,u_cl(:,1),'LineWidth',3)
plot(t,u_cl(:,2),'LineWidth',3)
plot(t,u_cl(:,3),'LineWidth',3)
legend('u_{r1}','u_{r2}','u_{r3}')
xlabel('Time [s]','FontSize',20)
ylabel('Torque [Nm]');



