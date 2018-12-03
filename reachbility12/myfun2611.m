function f = myfun2611(x)
%%
global xa_k ya_k fai_k vx_k vy_k r_k refx refy xin yuub yllb xa_kk ya_kk fai_kk vx_kk vy_kk r_kk
%%
%x(1) --> global x position -->x_a(k-1)--xa_k
%x(2) --> global y position -->y_a(k-1)--ya_k
%x(3) --> yaw angle Fai -->Fai(k-1)--fai_k
%x(4) --> vx vehicle x velcotiy -->vx(k-1)--vx_k
%x(5) --> vy vehicle y velocity -->vy(k-1)--vy_k
%x(6) --> yaw rate  --> r(k-1)--r_k--r_k
% u --> steering angle-->what i want--x
% x(7) :==u
%%
%r(1) -->Xd(k)  reference global x position  :==x(8)--refx
%r(2) -->Yd(k)  reference global y position --refy
%r(3) -->Xu(k)  reference x upper boundary --xin
%r(4) -->Yu(k)  reference y upper boundary --yuub
%r(5) -->Xl(k)  reference x lower boundary--xin
%r(6) -->Yl(k)  reference y lower boundary
%%
m=1298.9; %kg mass
lf=1; %m distance of CG from the front axle
lr=1.454; %m distance of CG from the rear axle
Cf=120000; %cornering stiffness of the tyre
Cr=120000; %cornering stiffness of the tyre
Iz=1627;%kgm^2 vehicle moment of inertial about yaw axle
Rw=0.35; %m wheel radius
dt=0.01; %s t(k) current time step t(k-1) time step of previous time
%%
a1=10000; %weight factor for optimizatio problem
b1=0;
b2=0;
b3=0.1;
%%
af=(vy_kk+lr*r_kk)/vx_kk-x(1);
ar=(vy_kk-lr*r_kk)/vx_kk;
Fcf=-Cf*af;
Fcr=-Cr*ar;
vx_k=vx_kk+(vy_kk*r_kk-2/m*(Fcf*sin(x(1))-x(2)/Rw))*dt;
vy_k=vy_kk+(-vx_kk*r_kk+2/m*(Fcf*cos(x(1))+Fcr))*dt;
r_k=r_kk+2/Iz*(lf*Fcf-lr*Fcr)*dt;
fai_k=fai_kk+r_k*dt;
vyg_k=vx_k*sin(fai_k)+vy_k*cos(fai_k);
vxg_k=vx_k*cos(fai_k)-vy_k*sin(fai_k);
ya_k=ya_kk+vyg_k*dt;
xa_k=xa_kk+vxg_k*dt;
f1=a1*((xa_k-refx)^2+(ya_k-refy)^2);
f2=b1*(1/((xa_k-xin)^2+(ya_k-yuub)^2))^2;
f3=b2*(1/((xa_k-xin)^2+(ya_k-yllb)^2))^2;
f4=b3*(r_k-r_kk)/dt;
f=f1+f2+f3+f4;
