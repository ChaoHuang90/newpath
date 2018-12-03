function limit = cones29three(manoeuvre,ref)
dx=0.1;
x=[ref.XX(1):dx:ref.XX(end)]';
limit.xinit=x;
x0=ref.XX(1);
y0=ref.YY(1);
x1 = x0 + manoeuvre.l1;
x12= x1-8; %6.5 initial value
x2 = x1 + manoeuvre.l2;
x3 = x2 + manoeuvre.l3;
x4 = x3 + manoeuvre.l4;
x5 = x4 + manoeuvre.l5;
x6 = x5 + manoeuvre.l6;
yl1 = y0-manoeuvre.w1/2;
yr1 = y0+manoeuvre.w1/2;
yl12= y0-manoeuvre.w1/2;
yr12= y0+manoeuvre.w1*3/2
yl2 = yl1;
yr2 = yr1+manoeuvre.w2/2;
yr3 = yr2;
yl3 = yr3-manoeuvre.w3;
yr4 = yr3;
yl4 = yr4-manoeuvre.w4;
yl5 = yl4;
yr5 = yl5+manoeuvre.w5;
yl6 = yl5;
yr6 = yl6+manoeuvre.w6;
if (nargin < 1)
    limit.X = [ x0; x12; x12;x1; x1; x1; x2; x2; x3; x3; x4; x4; x5; x5; x6];
limit.Y_lhs = [yl1;yl1;yl12;yl2;yl2;yl3;yl3;yl4;yl4;yl5;yl5;yl6;yl6];
limit.Y_rhs = [yr1;yr1;yr12;yr2;yr2;yr3;yr3;yr4;yr4;yr5;yr5;yr6;yr6];
plot(limit.X, [limit.Y_lhs,limit.Y_rhs])
else
if ((size(x,1) == 1) & (size(x,2) > 1))
x = x'
end
limit.X = x;
limit.Y_lhs(:,1) ...
= (x <= x1) .* yl1 ...
+ ((x > x1) & (x <= x2)) .* yl2 ...
+ ((x > x2) & (x <= x3)) .* yl3 ...
+ ((x > x3) & (x <= x4)) .* yl4 ...
+ ((x > x4) & (x <= x5)) .* yl5 ...
+ ((x > x5) & (x <= x6)) .* yl6 ...
+ (x > x6) .* yl6;
limit.Y_rhs(:,1) ...
= (x <= x12) .* yr12 ... % the orignal is yr1, now I changed to 5.25
+ ((x > x12) & (x <= x1)) .* yr1 ...
+ ((x > x1) & (x <= x2)) .* yr2 ...
+ ((x > x2) & (x <= x3)) .* yr3 ...
+ ((x > x3) & (x <= x4)) .* yr4 ...
+ ((x > x4) & (x <= x5)) .* yr5 ...
+ ((x > x5) & (x <= x6)) .* yr6 ...
+ (x > x6) .* yr6;
end