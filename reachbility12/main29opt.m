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
parameters.va=1.5;
dynamic_timestep=59;
%%
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
belta_max=0.8;
X0=[0;0];
%%
for j=0:0.1:dynamic_timestep
parameters.t=j;
manoeuvre = define_manoeuvre29three(parameters);
[ref] = referencetrajectory_arc29three(manoeuvre,vehicle,parameters);
limit = cones29three(manoeuvre,ref); % calls define manoeuvre
ylb = limit.Y_lhs;
yub = limit.Y_rhs;
[aw,bw]=size(ylb);
%%
w=fix(10*j)+1;
refx=ref.Xe(w);
refy=ref.Ye(w);
xin=limit.xinit(w);
yuub=yub(w);
yllb=ylb(w);
%plot(refx,refy,'dk');
hold on
%%
 aut.Xe(:,w)=xa_k;
 aut.Ye(:,w)=ya_k;
 aut.Psi(:,w)=fai_k;
 aut.Vx(:,w)=vx_k;
 aut.Vy(:,w)=vy_k;
 aut.dPsi(:,w)=r_k;
 reff.X(:,w)=refx;
 reff.Y(:,w)=refy;
 reff.xin(:,w)=xin;
 reff.yuub(:,w)=yuub;
 ref.yllb(:,w)=yllb;
 %%
 ant=atan((vy_k+vehicle.lf*r_k)/vx_k);
 x=fmincon(@myfun2911,X0,[],[],[],[],[-0.6109,160],[0.6109,300]);
 actual(:,w)=x;
 X0=x;
[xa_kk,ya_kk,fai_kk,vx_kk,vy_kk,r_kk,f] = myvehicle195(aut.Xe(:,w), aut.Ye(:,w),aut.Psi(:,w),aut.Vx(:,w),aut.Vy(:,w),aut.dPsi(:,w),x,reff.X(:,w),reff.Y(:,w),reff.xin(:,w),reff.yuub(:,w),ref.yllb(:,w));
aut.f(:,w)=f;
plot(xa_kk,ya_kk,'+r')
%refff=plot(aut.Xe,aut.Ye,':r','LineWidth',2);
hold on
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
%basic=plot(ref.Xe,ref.Ye,'--b','LineWidth',2);
%%
xlabel('Position X [m]');
ylabel('Position Y [m]');
pause(0.1)


if j<dynamic_timestep
delete(Car2);
delete(c2w1);
delete(c2w2);
delete(c2w3);
delete(c2w4);
delete(Boudary_u);
%delete(basic);
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
%delete(refff);
end

%axis([0 60 -2.1 6])
end
