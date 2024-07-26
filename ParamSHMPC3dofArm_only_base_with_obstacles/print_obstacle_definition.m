%% This script is used to visualized the robot and the obstacles, to setup the SHMPC

clc,close all,clear all

%Plot the robot in the initial configuration
xr = [deg2rad(90);deg2rad(90);deg2rad(90)];
xx2 = [zeros(3,1);xr;zeros(3,1);zeros(3,1)];
close all
print_system_config

%Add the obstacles

obs_diam = 0.5; obs_x = 1.2; obs_y = -0.3;
ang=0:0.005:2*pi;
r = obs_diam/2;  % obstacle radius
xp_obs=r*cos(ang);
yp_obs=r*sin(ang);

rob_diam = sqrt(2);
r = rob_diam/2;  % obstacle radius
xp_r=r*cos(ang);
yp_r=r*sin(ang);


plot(2.2+xp_r,yp_r,'--b'); % plot robot circle
plot(obs_x+xp_obs,obs_y+yp_obs,'--r'); % plot robot circle