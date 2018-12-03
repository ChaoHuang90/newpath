%%
%main file for reference path
% edit by Chao Huang 19:40 16/11/2018
% x0 --> initial value of vehicle states
%%
clc;clear;
close all
%%
%vehicle parameter
vehicle.lf=2.3; % m
vehicle.lr=2.25;
vehicle.w=1.81;
%%
parameters.mu=1;
parameters.spec='obstacle';
parameters.Vx=10;
parameters.va=5;
parameters.t=1;
manoeuvre = define_manoeuvre1611(parameters);
%%
[ref] = referencetrajectory_arc26(manoeuvre,vehicle,parameters);
%%
%plot figures road condition
lf=1; %m distance of CG from the front axle
lr=1.454; %m distance of CG from the rear axle
w=1.81; % vehicle width
spec='obstacle'
X0 = manoeuvre.l0; % m
X1 = manoeuvre.l1 + X0; % m
X2 = manoeuvre.l2 + X1; % m
X3 = manoeuvre.l3 + X2; % m
X4 = manoeuvre.l4 + X3; % m
X5 = manoeuvre.l5 + X4; % m
Y1 = 0.0; % m
Y3 = Y1 ...
- manoeuvre.w1/2 ...
+ manoeuvre.w2 ...
- manoeuvre.w3/2;
Y5 = Y3 ...
+ manoeuvre.w3/2 ...
- manoeuvre.w4 ...
+ manoeuvre.w5/2; % m
dx = 0.1;
xinit =[X0:dx:X5+3]';
limit = cones2611(manoeuvre, xinit, X0, Y1); % calls define manoeuvre
ylb = limit.Y_lhs;
yub = limit.Y_rhs;
L = length(xinit)-1;
% clearancef
ylb = ylb + 0*0.15;
yub = yub - 0*0.15;
[aw,bw]=size(ylb);
Road=plot(xinit,-2*ones(aw,1),'k','LineWidth',5)
legend({'Roadside'})
hold on
Bou=plot(xinit,ylb, '--b') %lower bounds
plot(xinit,5.5*ones(aw,1),'k','LineWidth',5)
hold on
plot( xinit,yub,'--b') % upper bounds
centre=plot(xinit,1.75*ones(aw,1),'--w','LineWidth',5);
%%
hold on
half_w=2.25; %haff width of the vehicle
% Blue car
% make it adaptive to the road information
xve1=-0.85;
xve2=0.85;
yve1=(X3+X2)/2-half_w;
yve2=(X3+X2)/2+half_w;
xve = [xve1, xve2, xve2, xve1, xve1];
yve = [yve1, yve1, yve2, yve2, yve1];
% plot(yve,xve,'b-','LineWidth',3);
Car1=fill(yve,xve,'b')
hold on
v1c1=[-0.85 (X3+X2)/2-half_w/2];
R1=0.1;
R2=0.5;
tvc=0:0.1:2*pi;
v1wheel1=[R1*cos(tvc)+v1c1(1); R2*sin(tvc)+v1c1(2)];
v1wheel1=[v1wheel1 v1wheel1(:,1)];
fill(v1wheel1(2,:),v1wheel1(1,:),'k')
v1c1=[0.85 (X3+X2)/2-half_w/2];
tvc=0:0.1:2*pi;
v1wheel1=[R1*cos(tvc)+v1c1(1); R2*sin(tvc)+v1c1(2)];
v1wheel1=[v1wheel1 v1wheel1(:,1)];
fill(v1wheel1(2,:),v1wheel1(1,:),'k')
v1c1=[-0.85 (X3+X2)/2+half_w/2];%%%16.5-15.725=0.7750;20.275-0.7750=19.5000
tvc=0:0.1:2*pi;
v1wheel1=[R1*cos(tvc)+v1c1(1); R2*sin(tvc)+v1c1(2)];
v1wheel1=[v1wheel1 v1wheel1(:,1)];
fill(v1wheel1(2,:),v1wheel1(1,:),'k')
v1c1=[0.85 (X3+X2)/2+half_w/2];%%%16.5-15.725=0.7750;20.275-0.7750=19.5000
tvc=0:0.1:2*pi;
v1wheel1=[R1*cos(tvc)+v1c1(1); R2*sin(tvc)+v1c1(2)];
v1wheel1=[v1wheel1 v1wheel1(:,1)];
fill(v1wheel1(2,:),v1wheel1(1,:),'k')
%% 
%Pink car
%make it adaptive to the road information
xve3=2.6;% constant
xve4=4.4;% constant
yve3=(X5+X4)/2-half_w;
yve4=(X5+X4)/2+half_w;
xve2 = [xve3, xve4, xve4, xve3, xve3];
yve2 = [yve3, yve3, yve4, yve4, yve3];
Car2=fill(yve2,xve2,'m')
v1c1=[2.6 (X5+X4)/2-half_w/2];%%%16.5-15.725=0.7750;20.275-0.7750=19.5000
tvc=0:0.1:2*pi;
v1wheel1=[R1*cos(tvc)+v1c1(1); R2*sin(tvc)+v1c1(2)];
v1wheel1=[v1wheel1 v1wheel1(:,1)];
fill(v1wheel1(2,:),v1wheel1(1,:),'k')
v1c1=[4.4 (X5+X4)/2-half_w/2];%%%16.5-15.725=0.7750;20.275-0.7750=19.5000
tvc=0:0.1:2*pi;
v1wheel1=[R1*cos(tvc)+v1c1(1); R2*sin(tvc)+v1c1(2)];
v1wheel1=[v1wheel1 v1wheel1(:,1)];
fill(v1wheel1(2,:),v1wheel1(1,:),'k')
v1c1=[2.6 (X5+X4)/2+half_w/2];%%%16.5-15.725=0.7750;20.275-0.7750=19.5000
tvc=0:0.1:2*pi;
v1wheel1=[R1*cos(tvc)+v1c1(1); R2*sin(tvc)+v1c1(2)];
v1wheel1=[v1wheel1 v1wheel1(:,1)];
fill(v1wheel1(2,:),v1wheel1(1,:),'k')
v1c1=[4.4 (X5+X4)/2+half_w/2];%%%16.5-15.725=0.7750;20.275-0.7750=19.5000
tvc=0:0.1:2*pi;
v1wheel1=[R1*cos(tvc)+v1c1(1); R2*sin(tvc)+v1c1(2)];
v1wheel1=[v1wheel1 v1wheel1(:,1)];
fill(v1wheel1(2,:),v1wheel1(1,:),'k')
hold on
basic=plot(ref.Xe,ref.Ye,'--y','LineWidth',2);
%%
%optimization problem with variable vx and vy
% the input is ax and front wheel angle
global xa_k ya_k fai_k vx_k vy_k r_k refx refy xin yuub yllb xa_kk ya_kk fai_kk vx_kk vy_kk r_kk
xa_k=0;
ya_k=0;
fai_k=0;
vx_k=10;
vy_k=0;
r_k=0;
xa_kk=0;
ya_kk=0;
fai_kk=0;
vx_kk=10;
vy_kk=0;
r_kk=0;
%%
%reference signal
refx=ref.Xe(1);
refy=ref.Ye(1);
xin=xinit(1);
yuub=yub(1);
yllb=ylb(2);
%%
belta_max=0.5;
X0=[0;0];
for i=1:length(ylb)-1
   %for i=1
 %%
 aut.Xe(:,i)=xa_k;
 aut.Ye(:,i)=ya_k;
 aut.Psi(:,i)=fai_k;
 aut.Vx(:,i)=vx_k;
 aut.Vy(:,i)=vy_k;
 aut.dPsi(:,i)=r_k;
 reff.X(:,i)=refx;
 reff.Y(:,i)=refy;
 reff.xin(:,i)=xin;
 reff.yuub(:,i)=yuub;
 ref.yllb(:,i)=yllb;
 %%
 ant=atan((vy_k+lf*r_k)/vx_k);
 x=fmincon(@myfun195,X0,[],[],[],[],[-0.6109,160],[0.6109,300]);
 %x=fmincon(@myfun192,X0,[],[],[],[],[-pi/5],[pi/5); % vx is constant
 actual(:,i)=x;
 refx=ref.Xe(i+1);
 refy=ref.Ye(i+1);
 xin=xinit(i+1);
 yuub=yub(i+1);
 yllb=ylb(i+1);
 X0=x;
[xa_kk,ya_kk,fai_kk,vx_kk,vy_kk,r_kk,f] = myvehicle195(aut.Xe(:,i), aut.Ye(:,i),aut.Psi(:,i),aut.Vx(:,i),aut.Vy(:,i),aut.dPsi(:,i),x,reff.X(:,i),reff.Y(:,i),reff.xin(:,i),reff.yuub(:,i),ref.yllb(:,i));
aut.f(:,i)=f;
end
hold on
reff=plot(aut.Xe,aut.Ye,':r','LineWidth',2);
hold on
plot(ref.X,ref.Y,'db');
for i=1:length(ref.X)
text(ref.X(i),ref.Y(i),['P' int2str(i)],'fontsize',15)
end
%%
%plot road information
legend([Road,Bou,centre,Car1,Car2,basic,reff],'Road side','Boundary','Centre Line','Obstacle1','Obstacle2','Reference Path','Trajectory');
%egend boxoff;
xlabel('Position X [m]');
ylabel('Position Y [m]');
set(gca,'color','[.7,.7,.7]')% background looks like a road