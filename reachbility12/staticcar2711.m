function s_vehicle=staticcar2711(half_w,ref)
%%
% static vehicle
% Chao Huang 27/11/2018
xve1=-0.85; % fixed
xve2=0.85;
%%
X1=ref.XX(2);
X2=ref.XX(3);
X3=ref.XX(4);
X4=ref.XX(5);
X5=ref.XX(6);
yve1=(X3+X2)/2-half_w;
yve2=(X3+X2)/2+half_w;
xve = [xve1, xve2, xve2, xve1, xve1];
yve = [yve1, yve1, yve2, yve2, yve1];
%%
s_vehicle.yve=yve;
s_vehicle.xve=xve;
%%
v1c1=[-0.85 (X3+X2)/2-half_w/2];
R1=0.1;
R2=0.5;
tvc=0:0.1:2*pi;
v1wheel1=[R1*cos(tvc)+v1c1(1); R2*sin(tvc)+v1c1(2)];
v1wheel1=[v1wheel1 v1wheel1(:,1)];
%%
s_vehicle.v1wheel1=v1wheel1;
%%
v1c2=[0.85 (X3+X2)/2-half_w/2];
tvc=0:0.1:2*pi;
v1wheel2=[R1*cos(tvc)+v1c2(1); R2*sin(tvc)+v1c2(2)];
v1wheel2=[v1wheel2 v1wheel2(:,1)];
%%
s_vehicle.v1wheel2=v1wheel2;
%%
v1c3=[-0.85 (X3+X2)/2+half_w/2];%%%16.5-15.725=0.7750;20.275-0.7750=19.5000
tvc=0:0.1:2*pi;
v1wheel3=[R1*cos(tvc)+v1c3(1); R2*sin(tvc)+v1c3(2)];
v1wheel3=[v1wheel3 v1wheel3(:,1)];
%%
s_vehicle.v1wheel3=v1wheel3;
%%
v1c4=[0.85 (X3+X2)/2+half_w/2];%%%16.5-15.725=0.7750;20.275-0.7750=19.5000
tvc=0:0.1:2*pi;
v1wheel4=[R1*cos(tvc)+v1c4(1); R2*sin(tvc)+v1c4(2)];
v1wheel4=[v1wheel4 v1wheel4(:,1)];
%%
s_vehicle.v1wheel4=v1wheel4;
end