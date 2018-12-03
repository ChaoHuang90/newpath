%main file for reference path
% edit by Chao Huang 19:40 16/11/2018
%x0 --> initial value of vehicle states
%x(1-6) --> vehicle status
%x(7) -->front wheel angle
%x(8-14) -->road condition which is known
%%
clc;clear;close all
%%
%vehicle parameter
vehicle.lf=2.3; % m
vehicle.lr=2.25;
vehicle.w=1.81;
%%
parameters.mu=1;
parameters.spec='obstacle';
parameters.Vx=10;
parameters.va=3;
for i=1:30
parameters.t=i;
manoeuvre = define_manoeuvre29three(parameters);
[ref] = referencetrajectory_arc29three(manoeuvre,vehicle,parameters);
%%
limit = cones29three(manoeuvre,ref); % calls define manoeuvre
ylb = limit.Y_lhs;
yub = limit.Y_rhs;
[aw,bw]=size(ylb);
Road=plot(limit.xinit,-2*ones(aw,1),'k','LineWidth',5)
hold on
plot(limit.xinit,5.5*ones(aw,1),'k','LineWidth',5)
hold on
Boudary_l=plot(limit.xinit,ylb, '--b') %lower bounds
hold on
Boudary_u=plot(limit.xinit,yub,'--b') % upper bounds
hold on
centre=plot(limit.xinit,1.75*ones(aw,1),'--w','LineWidth',5);
%%
% blue vehile
half_w=2.25; %haff width of the vehicle
s_vehicle=staticcar29three(half_w,ref)
Car1=fill(s_vehicle.yve,s_vehicle.xve,'b')
hold on
c1w1=fill(s_vehicle.v1wheel1(2,:),s_vehicle.v1wheel1(1,:),'k')
c1w2=fill(s_vehicle.v1wheel2(2,:),s_vehicle.v1wheel2(1,:),'k')
c1w3=fill(s_vehicle.v1wheel3(2,:),s_vehicle.v1wheel3(1,:),'k')
c1w4=fill(s_vehicle.v1wheel4(2,:),s_vehicle.v1wheel4(1,:),'k')
%% 
%Pink car
%dynamic vehicle
d_vehicle=dynamiccar29three(half_w,ref)
Car2=fill(d_vehicle.yve,d_vehicle.xve,'m')
hold on
c2w1=fill(d_vehicle.v2wheel1(2,:),d_vehicle.v2wheel1(1,:),'k')
c2w2=fill(d_vehicle.v2wheel2(2,:),d_vehicle.v2wheel2(1,:),'k')
c2w3=fill(d_vehicle.v2wheel3(2,:),d_vehicle.v2wheel3(1,:),'k')
c2w4=fill(d_vehicle.v2wheel4(2,:),d_vehicle.v2wheel4(1,:),'k')
%%
dy_vehicle=dynamiccar29threey(half_w,ref)
Car3=fill(dy_vehicle.yve,dy_vehicle.xve,'y')
hold on
c3w1=fill(dy_vehicle.v2wheel1(2,:),dy_vehicle.v2wheel1(1,:),'k')
c3w2=fill(dy_vehicle.v2wheel2(2,:),dy_vehicle.v2wheel2(1,:),'k')
c3w3=fill(dy_vehicle.v2wheel3(2,:),dy_vehicle.v2wheel3(1,:),'k')
c3w4=fill(dy_vehicle.v2wheel4(2,:),dy_vehicle.v2wheel4(1,:),'k')
%%
basic=plot(ref.Xe,ref.Ye,'--b','LineWidth',2);
%%
xlabel('Position X [m]');
ylabel('Position Y [m]');
pause(0.2)

if i<30
delete(Car2);
delete(c2w1);
delete(c2w2);
delete(c2w3);
delete(c2w4);
delete(Boudary_u);
delete(basic);
delete(Car1);
delete(c1w1);
delete(c1w2);
delete(c1w3);
delete(c1w4);
delete(Boudary_l);
delete(Car3);
delete(c3w1);
delete(c3w2);
delete(c3w3);
delete(c3w4);
end
axis([0 60 -2.1 6])
end
