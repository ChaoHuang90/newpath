%% matlab?
function [manoeuvre] = define_manoeuvre1611(parameters)
spec=parameters.spec;
time_step=0.1;
% Syntax:  
%    it is used to road information and 
%    static obstacle and dynamic obstacle
%
% Inputs:
%    spec='obstacle'
%    vehicle_width=1.8 m
% Outputs:
%    all the road information
%Author:       Chao Huang
%Written:      24-Nov-2018 
%Last update:  26-Nov-2018
switch (spec)
case 'obstacle'
manoeuvre.w1=3.5;
manoeuvre.w2=7;
manoeuvre.w3=3.5;
manoeuvre.w4=7;
manoeuvre.w5=3.5;
manoeuvre.l0=0;
manoeuvre.l1=6;
manoeuvre.l2=8;
manoeuvre.l3=10;
manoeuvre.l4=10;
manoeuvre.l5=12;
case 'ISO38881'
lane_offset = 3.5;
manoeuvre.w1 = 1.1*vehicle_width + 0.25;
manoeuvre.w3 = 1.2*vehicle_width + 0.25;
manoeuvre.w5 = 1.3*vehicle_width + 0.35;
manoeuvre.w2 = manoeuvre.w3 + lane_offset;
manoeuvre.w4 = manoeuvre.w2;
manoeuvre.l0 = 0*100;
manoeuvre.l1 = 15;
manoeuvre.l2 = 30;
manoeuvre.l3 = 25;
manoeuvre.l4 = 25;
manoeuvre.l5 = 30;
case 'ISO38882'
lane_offset = 1.0;
% manoeuvre.w1 = 1.1*vehicle_width + 0.25;
% manoeuvre.w3 = 1.0*vehicle_width + 1.00;
% manoeuvre.w5 = 1.3*vehicle_width + 0.25;
% manoeuvre.w5 = max(manoeuvre.w5,3);
% manoeuvre.w2 = manoeuvre.w1 + manoeuvre.w3 + lane_offset;
% manoeuvre.w4 = manoeuvre.w2;
manoeuvre.w1=1.98;
manoeuvre.w2=5.55;
manoeuvre.w3=2.57;
manoeuvre.w4=6.57;
manoeuvre.w5=3.00;
manoeuvre.l0 = 0*100.0;
manoeuvre.l1 = 12.0;
manoeuvre.l2 = 13.5;
manoeuvre.l3 = 11.0;
manoeuvre.l4 = 12.5;
manoeuvre.l5 = 12.0;
case 'single part 1'
lane_offset = 3.5;
manoeuvre.w1 = 1.1*vehicle_width + 0.25;
manoeuvre.w3 = 1.2*vehicle_width + 0.25;
manoeuvre.w2 = manoeuvre.w3 + lane_offset;
manoeuvre.w4 = manoeuvre.w3;
manoeuvre.w5 = manoeuvre.w3;
manoeuvre.l0 = 0*100.0;
manoeuvre.l1 = 15.0;
manoeuvre.l2 = 30.0;
manoeuvre.l3 = 25.0;
manoeuvre.l4 = 25.5;
manoeuvre.l5 = 15 + 15;
case 'single part 2'
lane_offset = 1.0;
manoeuvre.w1 = 1.1*vehicle_width + 0.25;
manoeuvre.w3 = 1.0*vehicle_width + 1.00;
manoeuvre.w2 = manoeuvre.w1 + manoeuvre.w3 + lane_offset;
manoeuvre.w4 = manoeuvre.w3;
manoeuvre.w5 = manoeuvre.w3;
manoeuvre.l0 = 0*100.0;
manoeuvre.l1 = 12.0;
manoeuvre.l2 = 13.5;
manoeuvre.l3 = 11.0;
manoeuvre.l4 = 12.5;
manoeuvre.l5 = 12.0;
case 'gentle lane change'
lane_offset = 3.5;
manoeuvre.w1 = 1.1*vehicle_width + 0.25;
manoeuvre.w3 = 1.2*vehicle_width + 0.25;
manoeuvre.w1 = manoeuvre.w1 * 1.50;
manoeuvre.w3 = manoeuvre.w3 * 1.50;
manoeuvre.w2 = manoeuvre.w3 + lane_offset;
manoeuvre.w4 = manoeuvre.w3;
manoeuvre.w5 = manoeuvre.w3;
manoeuvre.l0 = 0*100.0;
manoeuvre.l1 = 15.0;
manoeuvre.l2 = 30.0 * 1.5;
manoeuvre.l3 = 25.0;
manoeuvre.l4 = 25.5;
manoeuvre.l5 = 15 + 15;
otherwise
error('define_manoeuvre: spec is invalid');
end