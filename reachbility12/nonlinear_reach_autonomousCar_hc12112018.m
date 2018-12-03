function completed = nonlinear_reach_autonomousCar_hc12112018()

%------------- BEGIN CODE --------------
close all

dim = 6;

options.maxError = 0.5*ones(dim,1); % for comparison reasons


%set options --------------------------------------------------------------
options.tStart=0; %start time
options.tFinal=4.90; %final time
options.x0=[0; 0; 0; 10; 0 ; 0]; %initial state for simulation
options.R0 = zonotope([options.x0, 0.05*diag([1, 1, 1, 1, 1, 1])]); %initial state for reachability analysiszonotope([options.x0, diag([0.20, 0.20])]); %max for 3rd order
options.timeStep=0.01; %time step size for reachable set computation
options.taylorTerms=10; %number of taylor terms for reachable sets
options.zonotopeOrder=20; %zonotope order

options.uTransVec = uTRans_hc_14112018();
options.u = 0;
options.U = zonotope([0*options.uTransVec(:,1), 0.05*diag([ones(1,1);zeros(6,1)])]);

options.advancedLinErrorComp = 0;
options.tensorOrder = 1;
options.reductionInterval = inf;
options.reductionTechnique = 'girard';
%--------------------------------------------------------------------------

%specify continuous dynamics-----------------------------------------------
vehicle = nonlinearSys(6,7,@vmodel_linear_controlled_hc14,options); %initialize van-der-Pol oscillator
%--------------------------------------------------------------------------

%compute reachable set 
tic
Rcont = reach(vehicle, options);
tComp = toc;
disp(['computation time of reachable set: ',num2str(tComp)]);

%create random simulations; RRTs would provide better results, but are
%computationally more demanding
runs = 60;
fractionVertices = 0.5;
fractionInputVertices = 0.5;
inputChanges = 6;
simRes = simulate_random(vehicle, options, runs, fractionVertices, fractionInputVertices, inputChanges);

%plot results--------------------------------------------------------------
for plotRun=1:2
    % plot different projections
    if plotRun==1
        projectedDimensions=[1 2];
        projectedReference=[2, 3];
    elseif plotRun==2
        projectedDimensions=[4 5];   
        projectedReference=[5, 6];
    elseif plotRun==3
        projectedDimensions=[5 6]; 
        projectedReference=[6, 7];
    end 
    
    figure;
    hold on

    %plot reachable sets 
    for i=1:length(Rcont)
        Zproj = project(Rcont{i}{1},projectedDimensions);
        Zproj = reduce(Zproj,'girard',3);
        Reachhc=plotFilled(Zproj,[1 2],[.8 .8 .8],'EdgeColor','none');
    end
   
    %plot initial set
    plotFilled(options.R0,projectedDimensions,'w','EdgeColor','k');
    
    %plot simulation results      
    %{
    for i=1:length(simRes.t)
        plot(simRes.x{i}(:,projectedDimensions(1)),simRes.x{i}(:,projectedDimensions(2)),'Color',0*[1 1 1]);
    end
    %}
    %plot reference trajectory
    Ref=plot(options.uTransVec(projectedReference(1),:),options.uTransVec(projectedReference(2),:),'r','LineWidth',2);

    %label plot
    xlabel(['x_{',num2str(projectedDimensions(1)),'}']);
    ylabel(['x_{',num2str(projectedDimensions(2)),'}']);
    
end
%--------------------------------------------------------------------------

%example completed
completed = 1;
%%
%{
lf = 2.3; % m
lr = 2.25;
w=1.81;
spec='obstacle'
manoeuvre = define_manoeuvre(spec, w);
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
- manoeuvre.w3/2; % m
Y5 = Y3 ...
+ manoeuvre.w3/2 ...
- manoeuvre.w4 ...
+ manoeuvre.w5/2; % m
dx = 0.1;
xinit =[X0:dx:X5+3]';
limit = cones(spec, xinit, X0, Y1, w); % c a l l s def ine manoeuvre
ylb = limit.Y_lhs;
yub = limit.Y_rhs;
L = length(xinit)-1;
% clearancef
ylb = ylb + 0*0.15;
yub = yub - 0*0.15;
[aw,bw]=size(ylb);
Road=plot(xinit,-2*ones(aw,1),'k','LineWidth',2)
legend({'Roadside'})
hold on
Bou=plot(xinit,ylb, '--b')
plot(xinit,5.5*ones(aw,1),'k','LineWidth',2)
hold on
plot( xinit,yub,'--b')
xve1=-0.85;
xve2=0.85;
yve1=15.725+1.2;
yve2=20.275+1.2;
xve = [xve1, xve2, xve2, xve1, xve1];
yve = [yve1, yve1, yve2, yve2, yve1];
% plot(yve,xve,'b-','LineWidth',3);
Car1=fill(yve,xve,'b')
hold on
v1c1=[-0.85 16.5+1.2];
R1=0.1;
R2=0.5;
tvc=0:0.1:2*pi;
v1wheel1=[R1*cos(tvc)+v1c1(1); R2*sin(tvc)+v1c1(2)];
v1wheel1=[v1wheel1 v1wheel1(:,1)];
fill(v1wheel1(2,:),v1wheel1(1,:),'k')
v1c1=[0.85 16.5+1.2];
tvc=0:0.1:2*pi;
v1wheel1=[R1*cos(tvc)+v1c1(1); R2*sin(tvc)+v1c1(2)];
v1wheel1=[v1wheel1 v1wheel1(:,1)];
fill(v1wheel1(2,:),v1wheel1(1,:),'k')
v1c1=[-0.85 19.5+1.2];%%%16.5-15.725=0.7750;20.275-0.7750=19.5000
tvc=0:0.1:2*pi;
v1wheel1=[R1*cos(tvc)+v1c1(1); R2*sin(tvc)+v1c1(2)];
v1wheel1=[v1wheel1 v1wheel1(:,1)];
fill(v1wheel1(2,:),v1wheel1(1,:),'k')
v1c1=[0.85 19.5+1.2];%%%16.5-15.725=0.7750;20.275-0.7750=19.5000
tvc=0:0.1:2*pi;
v1wheel1=[R1*cos(tvc)+v1c1(1); R2*sin(tvc)+v1c1(2)];
v1wheel1=[v1wheel1 v1wheel1(:,1)];
fill(v1wheel1(2,:),v1wheel1(1,:),'k')
xve3=2.6;
xve4=4.4;
yve3=33.725+4;
yve4=38.175+4;
xve2 = [xve3, xve4, xve4, xve3, xve3];
yve2 = [yve3, yve3, yve4, yve4, yve3];
Car2=fill(yve2,xve2,'m')
v1c1=[2.6 34.5+4];%%%16.5-15.725=0.7750;20.275-0.7750=19.5000
tvc=0:0.1:2*pi;
v1wheel1=[R1*cos(tvc)+v1c1(1); R2*sin(tvc)+v1c1(2)];
v1wheel1=[v1wheel1 v1wheel1(:,1)];
fill(v1wheel1(2,:),v1wheel1(1,:),'k')
v1c1=[4.4 34.5+4];%%%16.5-15.725=0.7750;20.275-0.7750=19.5000
tvc=0:0.1:2*pi;
v1wheel1=[R1*cos(tvc)+v1c1(1); R2*sin(tvc)+v1c1(2)];
v1wheel1=[v1wheel1 v1wheel1(:,1)];
fill(v1wheel1(2,:),v1wheel1(1,:),'k')
v1c1=[2.6 37.4+4];%%%16.5-15.725=0.7750;20.275-0.7750=19.5000
tvc=0:0.1:2*pi;
v1wheel1=[R1*cos(tvc)+v1c1(1); R2*sin(tvc)+v1c1(2)];
v1wheel1=[v1wheel1 v1wheel1(:,1)];
fill(v1wheel1(2,:),v1wheel1(1,:),'k')
v1c1=[4.4 37.4+4];%%%16.5-15.725=0.7750;20.275-0.7750=19.5000
tvc=0:0.1:2*pi;
v1wheel1=[R1*cos(tvc)+v1c1(1); R2*sin(tvc)+v1c1(2)];
v1wheel1=[v1wheel1 v1wheel1(:,1)];
fill(v1wheel1(2,:),v1wheel1(1,:),'k')
% legend({'Roadside','(X,Y )','(Xref,Yref)' , 'Boundary','Wheels',});
legend([Reachhc,Road,Bou,Ref,Car1,Car2],'Reachbility set','Road side','Boundary','Referenca Path','Obstacle1','Obstacle2');
% legend({'Roadside'})
legend boxoff;
xlabel('Position X [m]');
ylabel('Position Y [m]');
%axis([0, 46, -5, 10]);
%axis('ij');

%}
%------------- END OF CODE --------------

