 function [ref] = referencetrajectory_arc29(manoeuvre,vehicle,parameters)
%%
mu = parameters.mu;
spec = parameters.spec;
Vx = parameters.Vx;
v_a=parameters.va; %m/s^2 dynamics vehicle speed
 %% vehicle parameters
lf = vehicle.lf;  % m
lr = vehicle.lr;
w=vehicle.w;
%% manoeuvre specification
X0 = manoeuvre.l0; % m
X1 = manoeuvre.l1 + X0; % m
X2 = manoeuvre.l2 + X1; % m
X3 = manoeuvre.l3 + X2; % m
X4 = manoeuvre.l4 + X3; % m
X5 = manoeuvre.l5 + X4; % m
%disp('[X0,X1,X2,X3,X4,X5,X6]'); [X0,X1,X2,X3,X4,X5,X6]
Y1 = 0.0; % m
Y3 = Y1 ...
+ manoeuvre.w1/2 ...
- manoeuvre.w2 ...
+ manoeuvre.w3/2; % m
Y5 = Y3 ...
- manoeuvre.w3/2 ...
+ manoeuvre.w4 ...
- manoeuvre.w5/2; % m
%disp('[Y1,Y3,Y5]'); [Y1,Y3,Y5]
%% constants of nature
g = 9.81; % m/ s2
a = mu*g; % max acceleration
r = Vx^2/a % circular motion
%%
% p0 starting position **
% p1 end of first straight
% p2 first approach to first corner **
% p3 end of first turn
% p4 start of second turn
% p5 second approach to boundary **
% p6 end of second turn
% p7 midpoint between 2 and 3 circles
% p8 start of third turn
% p9 third approach to boundary **
% p10 end of third turn
% p11 end of the trajectory **
%%
% (Xa ,Ya) P1 end of straight , start of first arc
% (Xb ,Yb) P3 end of first arc, start of straight
% (Xc ,Yc) P4 end of straight ,start of second arc
% (Xd ,Yd) P6 end of second arc, start of straight
% (Xe ,Ye) P8 end of straight, start of third arc
% (Xf, Yf) P10 end of third arc , start of straight
% Unknown : Xp1 ,Xp3 ,Xp4 ,Xp6 ,Xp7 ,Xp8 ,Xp10
%%
Xp0 = xc;
Xp2 = X1 - w/2; 
Xp5 = X2 + w/2;
Xp9 = X3 - w/2;
Xp11 = X5;
% Unknown : Yp3 ,Yp4 ,Yp5 , Yp10 , Yp11 , Yp12
Yp0 = yc;
Yp1 = Y1;
Yp2 = Y3 - manoeuvre.w2/2 + w/2;
Yp3 = Y3;
Yp4 = Y3;
Yp5 = Y3-manoeuvre.w2/2 + w/2;
Yp9 = Y5 + manoeuvre.w4/2 - w/2;
Yp10 = Y5;
Yp11 = Y5;
%%
theta2 = acos((1-(Yp3-Yp2)/r));
theta3 = acos((1-(Yp4-Yp5)/r));
theta4 = acos((1-(Yp9-Yp10)/r));
Xp3 = Xp2 + r*sin(theta2);
Xp4 = Xp5 - r*sin(theta3);
Xp10 = Xp9 + r*sin(theta4);
% circle centres
% 
Xo1 = Xp3; Yo1 = Yp2-sqrt(r^2-(Xp2-Xp3)^2);
Xo2 = Xp4; Yo2 = Yp4 - r;
Xo3 = Xp10; Yo3 = Yp10 + r;
Xp7 = (Xo2+Xo3)/2; Yp7 = (Yo2+Yo3)/2;
%
Xp1=Xo1-sqrt(r^2-(Yp1-Yo1)^2);
% still need (Xp6 ,Yp6) (Xp8 ,Yp8)
phi12 = asin(2*r/sqrt((Xo3-Xo2)^2+(Yo3-Yo2)^2));
sigma12 = atan2((Yo3-Yo2),(Xo3-Xo2));
Xp6 = Xo2 + r*sin(phi12-sigma12); Yp6 = Yo2 + r*cos(phi12-sigma12);
Xp8 = Xo3 - r*sin(phi12-sigma12); Yp8 = Yo3 - r*cos(phi12-sigma12);
%
[xcirc,ycirc] = pol2cart(linspace(0,2*pi,100),r*ones(1,100));
xcirc1 = xcirc + Xo1; ycirc1 = ycirc + Yo1;
xcirc2 = xcirc + Xo2; ycirc2 = ycirc + Yo2;
xcirc3 = xcirc + Xo3; ycirc3 = ycirc + Yo3;
%clf; 
%figure(4);
%{
plot(...
[Xp0;Xp1;Xp2;Xp3;Xp4;Xp5;Xp7;Xp8;Xp9;Xp10;Xp11], ...
[Yp0;Yp1;Yp2;Yp3;Yp4;Yp5;Yp7;Yp8;Yp9;Yp10;Yp11], ...
'db',...
 xcirc1, ycirc1, '-k', ...
 xcirc2, ycirc2, '-k', ...
 xcirc3, ycirc3, '-k' ...
);
%}
%%
mp = +tan(sigma12-phi12);
Xa = Xp1; Ya = Yp1;
Xb = Xp3; Yb = Yp3;
Xc = Xp4; Yc = Yp4;
Xd = Xp6; Yd = Yp6;
Xe = Xp8; Ye = Yp8;
Xf = Xp10; Yf = Yp10;
Xc1 = Xo1; Yc1 = Yo1;
Xc2 = Xo2; Yc2 = Yo2;
Xc3 = Xo3; Yc3 = Yo3;
spacing = (X2-X1) / 100;
spacing = 0.1;
X = [xc:spacing:X5];
i00 = find((X<X0));
i11 = find((X>=X0)&(X<=Xa));
i12 = find((X>Xa)&(X<=Xb));
i22 = find((X>Xb)&(X<=Xc));
i23 = find((X>Xc)&(X<=Xd));
if (abs(Y3 - Y5) < eps)
i33 = find(X>Xd);
i34 = [];
i44 = [];
else
i33 = find((X>Xd)&(X<=Xe));
i34 = find((X>Xe)&(X<=Xf));
i44 = find((X>Xf));
end
Y(i00) = Y1;
Y(i11) = Y1;
Y(i12) = Yc1 + sqrt(r^2 - (X(i12)-Xc1).^2);
Y(i22) = Yb;
Y(i23) = Yc2 + sqrt(r^2 - (X(i23)-Xc2).^2);
Y(i33) = Yd+mp*(X(i33)-Xd); %%
Y(i34) = Yc3 - sqrt(r^2 - (X(i34)-Xc3).^2);
Y(i44) = Y5;
ref.Xe = X;
ref.Ye = Y;
ref.X=[Xp0;Xp1;Xp2;Xp3;Xp4;Xp5;Xp7;Xp8;Xp9;Xp10;Xp11];
ref.Y=[Yp0;Yp1;Yp2;Yp3;Yp4;Yp5;Yp7;Yp8;Yp9;Yp10;Yp11];
ref.Cx1=xcirc1;
ref.Cy1=ycirc1;
ref.Cx2=xcirc2;
ref.Cy2=ycirc2;
ref.Cx3=xcirc3;
ref.Cy3=ycirc3;
ref.XX=[X0;X1;X2;X3;X4;X5];
ref.YY=[Y1;Y3;Y5];       