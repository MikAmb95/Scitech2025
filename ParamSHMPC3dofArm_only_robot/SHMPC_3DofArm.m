% This script impements a SH-MPC for a 3DoF robotic arm on a spacecraft,
% Tha bese if floating and the arm has to reach a final point
clear all; close all; clc

% the system has 6 position (x,y,psi) for the chaser and (q1,q2,q3) robot
% joints. 

% Import CasADi v3.6.3
addpath('C:\Users\Administrator\Documents\ULB\Casadi')
import casadi.*

%Simualation Paramenters
T = 0.01; %samplig time [s] 
N = 100; % prediction horizon (number of steps)

%Intervals(:,1) = [0;10;20;30;40;50;60;70;80;90;100];
L = 10; %Length Interval
Intervals = [0:L:N]; %Define the sequence of intervals



% State constraints (position and velocity constraints)

%min
qmin = [-10 -10 -deg2rad(180) -deg2rad(180) -deg2rad(180) -deg2rad(180)];
qmin(7:12) = -inf;

%max
qmax = [10 10 deg2rad(170) deg2rad(150) deg2rad(150) deg2rad(150)];
qmax(7:12) = inf;

%Input constraints (6 torques)
tau_max =[20;20;20]; 
tau_min = -tau_max;



%Symbolic variable to build the Casadi solver 
xc = SX.sym('xc'); yc = SX.sym('yc'); psi = SX.sym('psi'); %base-position state 
q1 = SX.sym('q1'); q2 = SX.sym('q2'); q3 = SX.sym('q3'); %robot-position state
 
dxc = SX.sym('dxc'); dyc = SX.sym('dyc'); dpsi = SX.sym('dpsi'); %base-velocity state
dq1 = SX.sym('dq1'); dq2 = SX.sym('dq2'); dq3 = SX.sym('dq3'); %robot-velocity state

%pack states
states = [xc;yc;psi;q1;q2;q3;dxc;dyc;dpsi;dq1;dq2;dq3]; n_states = length(states);

%Simbolic variables for the system inputs
u1 = SX.sym('u1'); u2 = SX.sym('u2'); u3 = SX.sym('u3'); 

controls = [u1;u2;u3]; n_controls = length(controls);


Np = (ceil(N)/L); %initial number of paramenters
Np0 = Np;


global H 
H = zeros(N,Np);
for i = 0:(Np-1)
H((L*i)+1:L*(i+1),i+1) = ones(L,1);
end

H0 = H;

U = SX.sym('U',n_controls,Np); % Decision variables (controls)

Up = H*U';

P = SX.sym('P',n_states + n_states); % parameters (include the initial state and the reference state)

X = SX.sym('X',n_states,(N+1)); % A vector that represents the states over the optimization problem.

%Initialized variables to build the solver(s)
obj = 0; % Objective function
g = [];  % constraints vector

% Cost Function weigth matrices

Q = zeros(12,12); %state matix 
%Position:
Q(1,1) = 1e-4; Q(2,2) = 1e-4; Q(3,3) = 1e-4; Q(4,4) = 1e7;
Q(5,5) = 1e7; Q(6,6) = 1e7;
%Velocity:
Q(7,7) = 1e-4; Q(8,8) = 1e-3; Q(9,9) = 1e-3; Q(10,10) = 1e1;
Q(11,11) = 1e1; Q(12,12) = 1e1;

R = zeros(3,3); %input matrix
R(1,1) = 0.0005; R(2,2) = 0.0005; 
R(3,3) = 0.0005*1e9; 


st  = X(:,1); % initial state
g = [g;st-P(1:n_states)]; % initial condition constraints (the fist 12-states = X0)

obj_v =[];

%here we create the objective functions for each iteration and the equality
%constraints due to the system dynamic
Up = Up';
for k = 1:N
     k
    st = X(:,k);  con = Up(:,k);
    obj_v = [obj_v;(st-P(n_states+1:end))'*Q*(st-P(n_states+1:end)) + con'*R*con];
    st_next = X(:,k+1);
    f_value = FFS_dynamic_model(st,con);
    st_next_euler = st+ (T*f_value);
    g = [g;st_next-st_next_euler]; % compute constraints
end




% make the decision variable one column  vector
OPT_variables = [reshape(X,n_states*(N+1),1);reshape(U,n_controls*Np,1)];

% create the solver
nlp_prob = struct('f', sum(obj_v), 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =3;%0,3 (0 = no iter details)
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver_struc = struct;
solver_struc.solver{1} = nlpsol('solver', 'ipopt', nlp_prob,opts);

%build vector of limits constraints
args = struct;
ns = n_states; nc = n_controls;
[lg,ug,lx,ux] = sh_constraints(N,n_states,n_controls*Np,qmin,qmax,tau_min,tau_max);
args.lbg = lg; args.ubg = ug;
args.lbx = lx; args.ubx = ux;


%This is for the SH, we pre-create all the solvers
Nf = N;
g3 = [];
for i = 2:Nf 
        i
        t1 = tic;
        N = N - 1;
        H = H(2:end,:);
        Np_new = ceil(N/L);
        
        if Np_new < Np
            U = U(:,1:end-1);
        end
        Np = Np_new;
        
        mpciter = i-1;
        OPT_variables2 = [reshape(X(:,1:end-mpciter),n_states*(N+1),1);reshape(U,n_controls*Np,1)];        
        obj2 = sum(obj_v(1:end-mpciter,1));
        g3 = g(1:end-n_states*mpciter,:);       
        nlp_prob2 = struct('f', obj2, 'x', OPT_variables2, 'g', g3, 'p', P);
        solver = nlpsol('solver', 'ipopt', nlp_prob2,opts);
        solver_struc.solver{i} = solver;
end
N = Nf;

H = H0;
%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SET UP


% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------
t0 = 0;
x0 = zeros(ns,1);    % initial condition.
xs = [0;0;0;deg2rad(30);deg2rad(-30) ; deg2rad(25);zeros(6,1)]; % Reference posture.

xx(:,1) = x0; % xx contains the history of states
t(1) = t0;


u0 = zeros(n_controls,Np0);        % two control inputs for each robot
X0 = repmat(x0,1,N+1)'; % initialization of the states decision variables

sim_tim = 20; % Maximum simulation time

% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];
tsim = N;

tic
while(mpciter < tsim-2)
    
    
    ss_error = norm((x0-xs),2)
    
    
   if mpciter >= 1 
        %when we enter here the horizon is shrinking so we need to remove
        %some contraints
        N = N -1;
        Np0 = ceil(N/L);
        
        Np_new = ceil(N/L);
        
        if Np_new < ceil(Intervals(end)/L)
            flag = 1;
        end
        
        [lg,ug,lx,ux] = sh_constraints(N,n_states,n_controls*Np0,qmin,qmax,tau_min,tau_max);
        args.lbg = lg;
        args.ubg = ug;
        args.lbx = lx;
        args.ubx = ux;
        X0 = X0(1:N+1,:); 
        u0 = zeros(n_controls,Np0);     
     end
    
    
    
    args.p   = [x0;xs]; % set the values of the parameters vector
    % initial value of the optimization variables
    args.x0  = [reshape(X0',ns*(N+1),1);reshape(u0,n_controls*Np0,1)];
    
    solver = solver_struc.solver{mpciter+1}; %we call the new solver (pre-computed) at each iterataion
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    
    
    u = reshape(full(sol.x(ns*(N+1)+1:end))',nc,Np0)'; % get controls only from the solution
    %xx1(:,1:ns,mpciter+1)= reshape(full(sol.x(1:ns*(N+1)))',ns,N+1)'; % get solution TRAJECTORY
    
    up = MovingBlockStrategy(mpciter,Intervals,u);
    
    u_cl= [u_cl ; up(1,:)]; %store input
    t(mpciter+1) = t0;
    
    % Apply the control and shift the solution
    [t0, x0, u0] = shift_dynamics(T, t0, x0, up); %the function shift calls the system dynamics
    xx(:,mpciter+2) = x0;
    X0 = reshape(full(sol.x(1:ns*(N+1)))',ns,N+1)'; % get solution TRAJECTORY
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:);X0(end,:)];
    
    
    mpciter
    mpciter = mpciter + 1;
    H = H(2:end,:);
    
end;
toc

ss_error = norm((x0-xs),2)

print_system_config

figure(1)
hold on
plot(t,xx(1,1:end-1),t,xs(1)*ones(i,size(t,2)))
plot(t,xx(2,1:end-1),t,xs(2)*ones(i,size(t,2)))

xx(4,:) = xx(4,:)+deg2rad(90); 
xx(5,:) = xx(5,:)+deg2rad(-90);
xx(6,:) = xx(6,:)+deg2rad(-90);

figure(2)
hold on
grid on
%plot(t,rad2deg(xx(3,1:end-1)),t,rad2deg(xs(3))*ones(i,size(t,2)))
plot(t*10,rad2deg(xx(4,1:end-1)),t*10,rad2deg(xs(4)+(pi/2))*ones(1,size(t,2)),'LineWidth',3)
plot(t*10,rad2deg(xx(5,1:end-1)),t*10,rad2deg(xs(5)-(pi/2))*ones(1,size(t,2)),'LineWidth',3)
plot(t*10,rad2deg(xx(6,1:end-1)),t*10,rad2deg(xs(6)-(pi/2))*ones(1,size(t,2)),'LineWidth',3)
xlabel('Time [s]','FontSize',20)
legend('q_{r1}','q_{rd1}','q_{r2}','q_{rd2}','q_{r3}','q_{rd3}')
ylabel('Joint [rad]');





figure(3)
grid on
hold on
plot(t*10,u_cl(:,1),'LineWidth',3)
plot(t*10,u_cl(:,2),'LineWidth',3)
plot(t*10,u_cl(:,3)*140,'LineWidth',3)
legend('u_{r1}','u_{r2}','u_{r3}')
xlabel('Time [s]','FontSize',20)
ylabel('Torque [Nm]');


function [lbg,ubg,lbx,ubx] = sh_constraints(N,ns,Np,qmin,qmax,tau_min,tau_max)

lbg(1:ns*(N+1)) = 0;  % -1e-20  % Equality constraints
ubg(1:ns*(N+1)) = 0;  % 1e-20   % Equality constraints

for i =1:ns
    lbx(i:ns:ns*(N+1),1) = qmin(i); %state q1 lower bound
    ubx(i:ns:ns*(N+1),1) = qmax(i); %state q1 upper bound
end



for i =1:Np
    lbx(ns*(N+1)+i:Np:ns*(N+1)+Np,1) = tau_min(1); %u1 lower bound
    ubx(ns*(N+1)+i:Np:ns*(N+1)+Np,1) = tau_max(1); %v upper bound
end

end


function u_p = MovingBlockStrategy(i,Intervals,theta)

global H

Np = size(theta,1);
H_size = size(H,2);
u_p = H(1:end,(H_size-Np+1):end)*theta;

end

