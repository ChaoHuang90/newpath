function d_vehicle=dynamiccar29three(half_w,ref)
%% dynamic vehicle
% Chao Huang 27/11/2018
R1=0.1;
R2=0.5;
xve3=2.6;% constant
xve4=4.4;% constant
%%
X1=ref.XX(2);
X2=ref.XX(3);
X3=ref.XX(4);
X4=ref.XX(5);
X5=ref.XX(6);
%%
yve3=(X5+X4)/2-half_w;
yve4=(X5+X4)/2+half_w;
xve2 = [xve3, xve4, xve4, xve3, xve3];
yve2 = [yve3, yve3, yve4, yve4, yve3];
%%
d_vehicle.yve=yve2;
d_vehicle.xve=xve2;
v2c1=[2.6 (X5+X4)/2-half_w/2];%%%16.5-15.725=0.7750;20.275-0.7750=19.5000
tvc=0:0.1:2*pi;
v2wheel1=[R1*cos(tvc)+v2c1(1); R2*sin(tvc)+v2c1(2)];
v2wheel1=[v2wheel1 v2wheel1(:,1)];
%%
d_vehicle.v2wheel1=v2wheel1;
%%
v2c2=[4.4 (X5+X4)/2-half_w/2];%%%
tvc=0:0.1:2*pi;
v2wheel2=[R1*cos(tvc)+v2c2(1); R2*sin(tvc)+v2c2(2)];
v2wheel2=[v2wheel2 v2wheel2(:,1)];
%%
d_vehicle.v2wheel2=v2wheel2;
%%
v2c3=[2.6 (X5+X4)/2+half_w/2];%%%
tvc=0:0.1:2*pi;
v2wheel3=[R1*cos(tvc)+v2c3(1); R2*sin(tvc)+v2c3(2)];
v2wheel3=[v2wheel3 v2wheel3(:,1)];
%%
d_vehicle.v2wheel3=v2wheel3;
%%
v2c4=[4.4 (X5+X4)/2+half_w/2];%%%16.5-15.725=0.7750;20.275-0.7750=19.5000
tvc=0:0.1:2*pi;
v2wheel4=[R1*cos(tvc)+v2c4(1); R2*sin(tvc)+v2c4(2)];
v2wheel4=[v2wheel4 v2wheel4(:,1)];
%%
d_vehicle.v2wheel4=v2wheel4;
%%
end
