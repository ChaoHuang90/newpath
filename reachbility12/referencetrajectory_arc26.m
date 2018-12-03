 function [ref] = referencetrajectory_arc26(manoeuvre,vehicle,parameters)
%%
mu = parameters.mu;
spec = parameters.spec;
Vx = parameters.Vx;
v_a=parameters.va;%m/s^2 dynamics vehicle speed
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
- manoeuvre.w1/2 ...
+ manoeuvre.w2 ...
- manoeuvre.w3/2; % m
Y5 = Y3 ...
+ manoeuvre.w3/2 ...
- manoeuvre.w4 ...
+ manoeuvre.w5/2; % m
%disp('[Y1,Y3,Y5]'); [Y1,Y3,Y5]
%% constants of nature
g = 9.81; % m/ s2
a = mu*g; % max acceleration
r = Vx^2/a % circular motion
% p0 starting position
% p1 end of first straight
% p2 first approach to first corner
% p3 end of first turn
% p4 midpoint of first 2 circles
% p5 start of second turn
% p6 second approach to boundary
% p7 end of second turn
% p8 start of third turn
% p9 third approach to boundary
% p10 end of third turn
% p11 midpoint between 3rd and 4 th circles
% p12 start of forth turn
% p13 fourth approach to boundary
% p14 end of fourth turn
% (Xa ,Ya) P1 end of straight , start of first arc
% (Xb ,Yb) P3 end of first arc, start of straight
% (Xc ,Yc ) P5 end of straight ,start of second arc
% (Xd ,Yd) P7 end of second arc , start of straight
% (Xe ,Ye) P8 end of straight , start of third arc
% ( Xf , Yf ) P10 end of third arc , start of straight
% (Xg ,Yg) P12 end of straight , start of fourth arc
% (Xh ,Yh) P14 end of fourth arc , start of straight
% Unknown : Xp1 ,Xp3 ,Xp4 ,Xp5 ,Xp7 ,Xp8 ,Xp10 , Xp11 , Xp12 , Xp14
%   because we delete one arc, the order of the p is different.
Xp0 = 0;
Xp2 = X1 + w/2;
Xp6 = X2 - w/2;
Xp9 = X3 + w/2;
Xp13 = X4 - w/2;
Xp15 = X5;
% Unknown : Yp3 ,Yp4 ,Yp5 , Yp10 , Yp11 , Yp12
Yp0 = Y1;
Yp1 = Y1;
Yp2 = Y1 + manoeuvre.w1/2 - w/2;
Yp6 = Y3 - manoeuvre.w3/2 + w/2;
Yp7 = Y3;
Yp8 = Y3;
Yp9 = Y3 - manoeuvre.w3/2 + w/2;
Yp13 = Y5 + manoeuvre.w5/2 - w/2;
Yp14 = Y5;
Yp15 = Y5;
theta1 = acos((1-(Yp2-Yp1)/r));
theta2 = acos((1-(Yp7-Yp6)/r));
theta3 = acos((1-(Yp8-Yp9)/r));
theta4 = acos((1-(Yp13-Yp14)/r));
Xp1 = Xp2 - r*sin(theta1);
Xp7 = Xp6 + r*sin(theta2);
Xp8 = Xp9 - r*sin(theta3);
Xp14 = Xp13 + r*sin(theta4);
% circle cent res
Xo1 = Xp1; Yo1 = Yp1 + r;
Xo2 = Xp7; Yo2 = Yp7 - r;
Xo3 = Xp8; Yo3 = Yp8 - r;
Xo4 = Xp14; Yo4 = Yp14 + r;
Xp4 = (Xo1+Xo2)/2; Yp4 = (Yo1+Yo2)/2;
Xp11 = (Xo3+Xo4)/2; Yp11 = (Yo3+Yo4)/2;
% still need (Xp3 ,Yp3 ) (Xp5 ,Yp5 ) (Xp10 , Yp10) (Xp12 , Yp12)
phi12 = asin(2*r/sqrt((Xo2-Xo1)^2+(Yo2-Yo1)^2));
phi34 = asin(2*r/sqrt((Xo4-Xo3)^2+(Yo4-Yo3)^2));
sigma12 = atan2((Yo2-Yo1),(Xo2-Xo1));
sigma34 = atan2((Yo4-Yo3),(Xo4-Xo3));
Xp3 = Xo1 + r*sin(phi12+sigma12); Yp3 = Yo1 - r*cos(phi12+sigma12);
Xp5 = Xo2 - r*sin(phi12+sigma12); Yp5 = Yo2 + r*cos(phi12+sigma12);
Xp10 = Xo3 + r*sin(phi34-sigma34); Yp10 = Yo3 + r*cos(phi34-sigma34);
Xp12 = Xo4 - r*sin(phi34-sigma34); Yp12 = Yo4 - r*cos(phi34-sigma34);
[xcirc,ycirc] = pol2cart(linspace(0,2*pi,100),r*ones(1,100));
xcirc1 = xcirc + Xo1; ycirc1 = ycirc + Yo1;
xcirc2 = xcirc + Xo2; ycirc2 = ycirc + Yo2;
xcirc3 = xcirc + Xo3; ycirc3 = ycirc + Yo3;
xcirc4 = xcirc + Xo4; ycirc4 = ycirc + Yo4;
%clf; 
%figure(4);
%{
plot(...
[Xp0;Xp1;Xp2;Xp3;Xp4;Xp5;Xp7;Xp8;Xp9;Xp10;Xp11;Xp12;Xp13;Xp14;Xp15], ...
[Yp0;Yp1;Yp2;Yp3;Yp4;Yp5;Yp7;Yp8;Yp9;Yp10;Yp11;Yp12;Yp13;Yp14;Yp15], ...
'db',...
 xcirc1, ycirc1, '-k', ...
 xcirc2, ycirc2, '-k', ...
 xcirc3, ycirc3, '-k', ...
 xcirc4, ycirc4, '-k' ...
 );
%}
% text(Xp0,Yp0,'P1');
% text(Xo2,Yo2,'o2');
% text(Xo3,Yo3,'o3');
% text(Xo4,Yo4,'o4');
%{
title([spec, ' at ', num2str(Vx*3.6), ' km/hr']);
xlabel('Position XE [m]'); ylabel('Position YE [m]');
axis('ij');
ymin = floor(-1+min(min(min(Yo1,Yo2),Yo3),Yo4)/2)*2;
ymax = ceil(+1+max(max(max(Yo1,Yo2),Yo3),Yo4)/2)*2;
axis([X0,X5,ymin,ymax])
view([-90,90]);
% axis('equal')
% axis([0 80 -2.5 5.5]);
print -depsc �waypoints.eps�;
saveas(gcf,'waypoints.fig');
% figure(1);
%}
mp = +tan(sigma12+phi12);
mq = +tan(sigma34-phi34);
Xa = Xp1; Ya = Yp1;
Xb = Xp3; Yb = Yp3;
Xc = Xp5; Yc = Yp5;
Xd = Xp7; Yd = Yp7;
Xe = Xp8; Ye = Yp8;
Xf = Xp10; Yf = Yp10;
Xg = Xp12; Yg = Yp12;
Xh = Xp14; Yh = Yp14;
Xc1 = Xo1; Yc1 = Yo1;
Xc2 = Xo2; Yc2 = Yo2;
Xc3 = Xo3; Yc3 = Yo3;
Xc4 = Xo4; Yc4 = Yo4;
%{
disp('Xa,Xb,Xc,Xd,Xe,Xf,Xg,Xh');
% [Xa,Xb,Xc,Xd,Xe,Xf,Xg,Xh]
disp('Xc1,Yc1,Xc2,Yc2,Xc3,Yc3,Xc4,Yc4');
%}
% [Xc1,Yc1,Xc2,Yc2,Xc3,Yc3,Xc4,Yc4]
spacing = (X2-X1) / 100;
spacing = 0.1;
X = [0:spacing:X5+3];
i00 = find((X<X0));
i11 = find((X>=X0)&(X<=Xa));
i12 = find((X>Xa)&(X<=Xb));
i22 = find((X>Xb)&(X<=Xc));
i23 = find((X>Xc)&(X<=Xd));
if (abs(Y3 - Y5) < eps)
i33 = find(X>Xd);
i34 = [];
i44 = [];
i45 = [];
i55 = [];
else
i33 = find((X>Xd)&(X<=Xe));
i34 = find((X>Xe)&(X<=Xf));
i44 = find((X>Xf)&(X<=Xg));
i45 = find((X>Xg)&(X<=Xh));
i55 = find((X>Xh));
end
Y(i00) = Y1;
Y(i11) = Y1;
Y(i12) = Yc1 - sqrt(r^2 - (X(i12)-Xc1).^2);
Y(i22) = Yb + mp*(X(i22)-Xb);
Y(i23) = Yc2 + sqrt(r^2 - (X(i23)-Xc2).^2);
Y(i33) = Y3;
Y(i34) = Yc3 + sqrt(r^2 - (X(i34)-Xc3).^2);
Y(i44) = Yf + mq*(X(i44)-Xf);
Y(i45) = Yc4 - sqrt(r^2 - (X(i45)-Xc4).^2);
Y(i55) = Y5;
dY_dX(i00) = 0;
dY_dX(i11) = 0;
dY_dX(i12) = +(X(i12)-Xc1)./sqrt(r^2-(X(i12)-Xc1).^2);
dY_dX(i22) = mp;
dY_dX(i23) = -(X(i23)-Xc2)./sqrt(r^2-(X(i23)-Xc2).^2);
dY_dX(i33) = 0;
dY_dX(i34) = -(X(i34)-Xc3)./sqrt(r^2-(X(i34)-Xc3).^2);
dY_dX(i44) = mq;
dY_dX(i45) = +(X(i45)-Xc4)./sqrt(r^2-(X(i45)-Xc4).^2);
dY_dX(i55) = 0;
d2Y_dX2(i00) = 0;
d2Y_dX2(i11) = 0;
d2Y_dX2(i12) = +r^2./((r^2-(X(i12)-Xc1).^2).^1.5);
d2Y_dX2(i22) = 0;
d2Y_dX2(i23) = -r^2./((r^2-(X(i23)-Xc2).^2).^1.5);
d2Y_dX2(i33) = 0;
d2Y_dX2(i34) = -r^2./((r^2-(X(i34)-Xc3).^2).^1.5);
d2Y_dX2(i44) = 0;
d2Y_dX2(i45) = +r^2./((r^2-(X(i45)-Xc4).^2).^1.5);
d2Y_dX2(i55) = 0;
Psi = atan(dY_dX);
dPsi_dX = d2Y_dX2./(dY_dX.^2+1);
Psidot = dPsi_dX*Vx;
RoT(i00) = 0;
RoT(i11) = 0;
RoT(i12) = +r;
RoT(i22) = 0;
RoT(i23) = -r;
RoT(i33) = 0;
RoT(i34) = -r;
RoT(i44) = 0;
RoT(i45) = +r;
RoT(i55) = 0;
delta = zeros(size(X));
nz = find(RoT);
delta(nz) = atan((lf+lr)./RoT(nz));
fref = zeros(size(X));
ref.Xe = X;
ref.Ye = Y;
ref.Psi = Psi;
ref.Xdot = Vx*ones(size(X));
ref.Ydot = 0*ones(size(X));
ref.Psidot = Psidot;
ref.Xddot = 0*ones(size(X));
ref.Yddot = 0*ones(size(X));
ref.Psiddot = 0*ones(size(X));
ref.delta = delta;
ref.X=[Xp0;Xp1;Xp2;Xp3;Xp4;Xp5;Xp7;Xp8;Xp9;Xp10;Xp11;Xp12;Xp13;Xp14;Xp15];
ref.Y=[Yp0;Yp1;Yp2;Yp3;Yp4;Yp5;Yp7;Yp8;Yp9;Yp10;Yp11;Yp12;Yp13;Yp14;Yp15];
ref.Cx1=xcirc1;
ref.Cy1=ycirc1;
ref.Cx2=xcirc2;
ref.Cy2=ycirc2;
ref.Cx3=xcirc3;
ref.Cy3=ycirc3;
ref.Cx4=xcirc4;
ref.Cy4=ycirc4;
ref.XX=[X0;X1;X2;X3;X4;X5];
ref.YY=[Y1;Y3;Y5];