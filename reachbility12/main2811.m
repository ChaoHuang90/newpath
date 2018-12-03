%%
% main file for reference path
% edit by Chao Huang 19:40 16/11/2018
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
for i=1:30
parameters.t=i;
manoeuvre = define_manoeuvre2811(parameters);
%%
[ref] = referencetrajectory_arc28(manoeuvre,vehicle,parameters);
circle=plot(ref.X, ref.Y, 'db',ref.Cx1,ref.Cy1, '-k', ref.Cx2,ref.Cy2, '-k',ref.Cx3,ref.Cy3, '-k');
hold on
basic=plot(ref.Xe,ref.Ye,'--r','LineWidth',2);
hold on
%%
%plot figures road condition
limit = cones2811(manoeuvre,ref); % calls define manoeuvre
ylb = limit.Y_lhs;
yub = limit.Y_rhs;
[aw,bw]=size(ylb);
Road=plot(limit.xinit,-2*ones(aw,1),'k','LineWidth',5)
hold on
plot(limit.xinit,5.5*ones(aw,1),'k','LineWidth',5)
hold on
plot(limit.xinit,ylb, '--b') %lower bounds
hold on
Boudary=plot(limit.xinit,yub,'--b') % upper bounds
hold on
centre=plot(limit.xinit,1.75*ones(aw,1),'--w','LineWidth',5);
%%
% static vehile
half_w=2.25; %haff width of the vehicle
s_vehicle=staticcar2811(half_w,ref)
Car1=fill(s_vehicle.yve,s_vehicle.xve,'b')
hold on
fill(s_vehicle.v1wheel1(2,:),s_vehicle.v1wheel1(1,:),'k')
fill(s_vehicle.v1wheel2(2,:),s_vehicle.v1wheel2(1,:),'k')
fill(s_vehicle.v1wheel3(2,:),s_vehicle.v1wheel3(1,:),'k')
fill(s_vehicle.v1wheel4(2,:),s_vehicle.v1wheel4(1,:),'k')
%% 
%dynamic vehicle
d_vehicle=dynamiccar2811(half_w,ref)
Car2=fill(d_vehicle.yve,d_vehicle.xve,'m')
hold on
c2w1=fill(d_vehicle.v2wheel1(2,:),d_vehicle.v2wheel1(1,:),'k')
c2w2=fill(d_vehicle.v2wheel2(2,:),d_vehicle.v2wheel2(1,:),'k')
c2w3=fill(d_vehicle.v2wheel3(2,:),d_vehicle.v2wheel3(1,:),'k')
c2w4=fill(d_vehicle.v2wheel4(2,:),d_vehicle.v2wheel4(1,:),'k')
pause(0.2)

if i<30
delete(Car2);
delete(c2w1);
delete(c2w2);
delete(c2w3);
delete(c2w4);
delete(Boudary);
delete(basic);
delete(circle);
end
%}
end
%%
%optimization problem with variable vx and vy
% the input is ax and front wheel angle