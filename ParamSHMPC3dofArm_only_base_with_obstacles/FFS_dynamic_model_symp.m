
function out = FFS_dynamic_model_symp(x,tau)
    %run('robot_paramenters.m') %function with the dynamic paramenters of the system (robot+target)
    
    %simplify model system considering the robot in a constant
    %configuration
    
    % Characteristics of the robot---------------------------------------------
    a1=2; %length of the link #1 [m]
    a2=2; %length of the link #2 [m]
    a3=2; %length of the link #3 [m]
    ac1=a1/2; %CoM link #1 [m]
    ac2=a2/2; %CoM link #2 [m]
    ac3=a3/2; %CoM link #3 [m]
    m1=2; %mass link #1 [kg]
    m2=2; %mass link #2 [kg]
    m3=2; %mass link #3 [kg]
    mb=20; %mass base #1 [kg]
    mp=1; %mass EE [kg]
    Izz1=1/12*m1*a1^2; %moment of inertia link #1 [kg*m^2]
    Izz2=1/12*m2*a2^2; %moment of inertia link #2 [kg*m^2]
    Izz3=1/12*m3*a3^2; %moment of inertia link #3 [kg*m^2]
    W1=0.5; % (the base is modeled as a rectangular) side #1 [m]
    W2=0.5; % (the base is modeled as a rectangular) side #2 [m]
    Izzb=1/12*mb*(4*W1^2+4*W2^2); % moment of inertia base [kg*m^2]
    ex=W1*0-0.1*0; %where the robot is connected to the base (x-direction)
    ey=W2*0-0.1*0; %where the robot is connected to the base (y-direction)

    % States---------------------------------------------------------------
    
    %Position
    xc=x(1); yc=x(2); psi=x(3);
    q1=0;    q2=0; q3=0;
    
    
    %Velocity
    dxc=x(4);    dyc=x(5);    dpsi=x(6);
    dq1=0;    dq2=0;    dq3=0;
    
    
   
    
   
    % Model----------------------------------------------------------------
    m11=m1+m2+m3+mb+mp;
    m12=0;
m13 = ey*m1*cos(psi) + ey*m2*cos(psi) + ey*m3*cos(psi) + ey*mp*cos(psi) - a1*m2*sin(psi) - a1*m3*sin(psi) - a2*m3*sin(psi) - ac1*m1*sin(psi) - ac2*m2*sin(psi) - ac3*m3*sin(psi) - a1*mp*sin(psi) - a2*mp*sin(psi) - a3*mp*sin(psi) - ex*m1*sin(psi) - ex*m2*sin(psi) - ex*m3*sin(psi) - ex*mp*sin(psi);
m14 = -sin(psi)*(a1*m2 + a1*m3 + a2*m3 + ac1*m1 + ac2*m2 + ac3*m3 + a1*mp + a2*mp + a3*mp);
m15 = -sin(psi)*(a2*m3 + ac2*m2 + ac3*m3 + a2*mp + a3*mp);   
m16 = -sin(psi)*(ac3*m3 + a3*mp);
m22 = m1 + m2 + m3 + mb + mp;
m23 = a1*m2*cos(psi) + a1*m3*cos(psi) + a2*m3*cos(psi) + ac1*m1*cos(psi) + ac2*m2*cos(psi) + ac3*m3*cos(psi) + a1*mp*cos(psi) + a2*mp*cos(psi) + a3*mp*cos(psi) + ex*m1*cos(psi) + ex*m2*cos(psi) + ex*m3*cos(psi) + ex*mp*cos(psi) + ey*m1*sin(psi) + ey*m2*sin(psi) + ey*m3*sin(psi) + ey*mp*sin(psi);
m24 = cos(psi)*(a1*m2 + a1*m3 + a2*m3 + ac1*m1 + ac2*m2 + ac3*m3 + a1*mp + a2*mp + a3*mp);
m25 = cos(psi)*(a2*m3 + ac2*m2 + ac3*m3 + a2*mp + a3*mp);
m26 = cos(psi)*(ac3*m3 + a3*mp);
m33 = Izz1 + Izz2 + Izz3 + Izzb + a1^2*m2 + a1^2*m3 + a2^2*m3 + ac1^2*m1 + ac2^2*m2 + ac3^2*m3 + a1^2*mp + a2^2*mp + a3^2*mp + ex^2*m1 + ex^2*m2 + ex^2*m3 + ey^2*m1 + ey^2*m2 + ey^2*m3 + ex^2*mp + ey^2*mp + 2*a1*a2*m3 + 2*a1*ac2*m2 + 2*a1*ac3*m3 + 2*a2*ac3*m3 + 2*a1*a2*mp + 2*a1*a3*mp + 2*a2*a3*mp + 2*a1*ex*m2 + 2*a1*ex*m3 + 2*a2*ex*m3 + 2*ac1*ex*m1 + 2*ac2*ex*m2 + 2*ac3*ex*m3 + 2*a1*ex*mp + 2*a2*ex*mp + 2*a3*ex*mp;
m34 = Izz1 + Izz2 + Izz3 + a1^2*m2 + a1^2*m3 + a2^2*m3 + ac1^2*m1 + ac2^2*m2 + ac3^2*m3 + a1^2*mp + a2^2*mp + a3^2*mp + 2*a1*a2*m3 + 2*a1*ac2*m2 + 2*a1*ac3*m3 + 2*a2*ac3*m3 + 2*a1*a2*mp + 2*a1*a3*mp + 2*a2*a3*mp + a1*ex*m2 + a1*ex*m3 + a2*ex*m3 + ac1*ex*m1 + ac2*ex*m2 + ac3*ex*m3 + a1*ex*mp + a2*ex*mp + a3*ex*mp;
m35 = Izz2 + Izz3 + a2^2*m3 + ac2^2*m2 + ac3^2*m3 + a2^2*mp + a3^2*mp + a1*a2*m3 + a1*ac2*m2 + a1*ac3*m3 + 2*a2*ac3*m3 + a1*a2*mp + a1*a3*mp + 2*a2*a3*mp + a2*ex*m3 + ac2*ex*m2 + ac3*ex*m3 + a2*ex*mp + a3*ex*mp;
m36 = Izz3 + ac3^2*m3 + a3^2*mp + a1*ac3*m3 + a2*ac3*m3 + a1*a3*mp + a2*a3*mp + ac3*ex*m3 + a3*ex*mp;
m44 = Izz1 + Izz2 + Izz3 + a1^2*m2 + a1^2*m3 + a2^2*m3 + ac1^2*m1 + ac2^2*m2 + ac3^2*m3 + a1^2*mp + a2^2*mp + a3^2*mp + 2*a1*a2*m3 + 2*a1*ac2*m2 + 2*a1*ac3*m3 + 2*a2*ac3*m3 + 2*a1*a2*mp + 2*a1*a3*mp + 2*a2*a3*mp;
m45 = Izz2 + Izz3 + a2^2*m3 + ac2^2*m2 + ac3^2*m3 + a2^2*mp + a3^2*mp + a1*a2*m3 + a1*ac2*m2 + a1*ac3*m3 + 2*a2*ac3*m3 + a1*a2*mp + a1*a3*mp + 2*a2*a3*mp;
m46 = Izz3 + ac3^2*m3 + a3^2*mp + a1*ac3*m3 + a2*ac3*m3 + a1*a3*mp + a2*a3*mp;
m55 = Izz2 + Izz3 + a2^2*m3 + ac2^2*m2 + ac3^2*m3 + a2^2*mp + a3^2*mp + 2*a2*ac3*m3 + 2*a2*a3*mp;
m56 = mp*a3^2 + a2*mp*a3 + m3*ac3^2 + a2*m3*ac3 + Izz3;
m66 = mp*a3^2 + m3*ac3^2 + Izz3;
c1 = -dpsi^2*(a1*m2*cos(psi) + a1*m3*cos(psi) + a2*m3*cos(psi) + ac1*m1*cos(psi) + ac2*m2*cos(psi) + ac3*m3*cos(psi) + a1*mp*cos(psi) + a2*mp*cos(psi) + a3*mp*cos(psi) + ex*m1*cos(psi) + ex*m2*cos(psi) + ex*m3*cos(psi) + ex*mp*cos(psi) + ey*m1*sin(psi) + ey*m2*sin(psi) + ey*m3*sin(psi) + ey*mp*sin(psi));
c2 = -dpsi^2*(a1*m2*sin(psi) - ey*m2*cos(psi) - ey*m3*cos(psi) - ey*mp*cos(psi) - ey*m1*cos(psi) + a1*m3*sin(psi) + a2*m3*sin(psi) + ac1*m1*sin(psi) + ac2*m2*sin(psi) + ac3*m3*sin(psi) + a1*mp*sin(psi) + a2*mp*sin(psi) + a3*mp*sin(psi) + ex*m1*sin(psi) + ex*m2*sin(psi) + ex*m3*sin(psi) + ex*mp*sin(psi));
c3 = 0;
c4 = dpsi^2*ey*(a1*m2 + a1*m3 + a2*m3 + ac1*m1 + ac2*m2 + ac3*m3 + a1*mp + a2*mp + a3*mp);
c5 = dpsi^2*ey*(a2*m3 + ac2*m2 + ac3*m3 + a2*mp + a3*mp);
c6 = dpsi^2*ey*(ac3*m3 + a3*mp);

 







    % Matrices-------------------------------------------------------------
    Mb=[m11,m12,m13;...
        m12,m22,m23;...
        m13,m23,m33];

    Mba=[m14,m15,m16;...
        m24,m25,m26;...
        m34,m35,m36];

    Ma=[m44,m45,m46;...
        m45,m55,m56;...
        m46,m56,m66];
    
    cb=[c1;c2;c3];
    ca=[c4;c5;c6];
    
    M=[Mb,Mba;Mba',Ma];
    c=[cb;ca];
    
    tau = [tau;zeros(3,1)];
    
    acc = inv(M)*(-c+tau);
    
    
    %acc = tau;
    
    out = [x(4:6);acc(1:3)]; 
    