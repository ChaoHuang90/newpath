function dx = vmodel_linear_controlled_hc(t,x,u)
%Bicycle Model with 
% - normal force equilibrium for pitching-moments
% - linear tyre model
% state x=[X,Y,psi,vx,vy,omega]
% input u=[delta,omega_f,omega_r]

%parameters: get parameters from p vector
    %body
    m = 2364;
    J = 4488;
    L = 2.7;
    l_F = 2.3;
    l_R = 2.25;
    h = 0.5;

    %street
    mu0 = 1;
    g = 9.81;
    dt=0.01;
    
    %tires
    %B=p(7),C=p(8) - paceijca parameter in ff_abs = sin(C*atan(B*sf_abs/mu0))
    C_F = 120000;
    C_R = 120000;

%state
    %position
    X = x(1); %#ok<NASGU>
    Y = x(2); %#ok<NASGU>
    psi = x(3);
    
    %velocity
    vx = x(4);
    vy = x(5);
    omega = x(6);
    
    %acceleration
    
    
%control action
% inputs are values of the state feedback matrix R, the reference state Xn,
% and the feedforward value W

R = [u(1) u(2) u(3) u(4) u(5) u(6)];
Xn = [u(7); u(8); u(9); u(10); u(11); u(12)];
W = [u(13)];
v = R*(Xn-x)+W;
        
%calculate normal forces


%side-slip
sf =(vy+l_F*omega)/vx-v;
sr =(vy-l_R*omega)/vx;
% omega is r

%forces
Fyf = -C_F  * sf;
Fyr = -C_R  * sr;


%ACCELERATIONS
dvx = 0;
dvy = (Fyf+Fyr)/m - vx * omega;
dvy=-(C_F+C_R)/vx/m*vy+((l_R*C_R-l_F*C_F)/m/vx-vx)*omega+C_F/m*v;
domega = (l_F * Fyf - l_R * Fyr)/J;


%position
    cp = cos(psi);sp = sin(psi);
    dx(1,1) = (cp * vx - sp * vy);
    dx(2,1) = (sp * vx + cp * vy);
    dx(3,1) = omega;
%velocity
    dx(4,1) = dvx;
    dx(5,1) = dvy;
    dx(6,1) = domega;
%front wheel angle
    %dx(7,1) = v(1);
