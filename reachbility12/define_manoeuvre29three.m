%% ??matlab??
function [manoeuvre] = define_manoeuvre29three(parameters)
mu = parameters.mu;
spec = parameters.spec;
Vx = parameters.Vx;
v_a=parameters.va;%m/s^2 dynamics vehicle speed
k=parameters.t;
time_step=0.1*k;
% Syntax:  
switch (spec)
case 'obstacle'
manoeuvre.w1=3.5;
manoeuvre.w2=7;
manoeuvre.w3=3.5;
manoeuvre.w4=7;
manoeuvre.w5=3.5;
manoeuvre.w6=7;
manoeuvre.l0=0;
manoeuvre.l1=9+v_a*time_step;
manoeuvre.l2=10-(1-2/3)*v_a*time_step;
manoeuvre.l3=8;
manoeuvre.l4=10+(4/3-2/3)*v_a*time_step;
manoeuvre.l5=8;
manoeuvre.l6=15-4/3*v_a*time_step;
otherwise
error('define_manoeuvre: spec is invalid');
end