%before running this file make sure all the following files are in same
%folder
%1. hmapsf_Initializ.m
%2. hmapsf.slx
%3. hmapsf_Plot.m

%This file creates a Heuristic map. distance metric as arc length of
%traversed path subjected to the non-holonomic constraints of a SAV
%running hmapsf_Initializ.m calls the simulink Stateflow model hmapsf.slx
%where SAV vehicle model and controller are forward simulated over linear
%reference paths generated between (0,0) and individul point in the grid
%spanned by x,y with ranges 0<=x<=50; -50<=y<=50; later on the obtained 
%results are mirrored to the left half plane and plotted using hmapsf_Plot.m.
% This map data will be used in clrrtsf_Initializ.slx which is a CLRRT
% planner model named clrrtsf.slx initializer.

clear
%set velocity -1 for revrese heuristic map; +1  for forward heuristic map
v_0 = 1;  
% Initialize vehicle model
L_0f = 3.8;  %(tractor wheelbase)
L_1f = 7.21; %(trailer wheel base)
L_0b = 0.48;  %(distance tractor rear axle-coupling point)
LD=10; %LOOKAHEAD DISTANCE
tstep=0.2; %setting step time for simulink model hmapsf.slx
% Initial Conditions  

if v_0>0
IC_theta1=(pi)/2;                  %enter initial semitrailer orientation for forward direction
gamma_1=0;                         %enter initial articulation angle
elseif v_0<0
IC_theta1=(3*pi)/2;                %enter initial semitrailer orientation  for reverse direction
gamma_1=0;                         %enter initial articulation angle    
end

IC_x1= 0;                                   % initial x-position of semi axle 
IC_y1 = 0;                               
IC_theta0 =gamma_1+IC_theta1;               % initial yaw angle of tractor [deg]



IC_x1f= L_1f*cos(IC_theta1)+IC_x1 ;         % initial x-position of semi axle 
IC_y1f =L_1f*sin(IC_theta1)+IC_y1 ;

IC_x0f= (L_0f-L_0b)*cos(IC_theta0)+IC_x1f ;                                                         
IC_y0f =(L_0f-L_0b)*sin(IC_theta0)+IC_y1f ;

IC_x0 =   IC_x0f-L_0f*cos(IC_theta0);       % initial x-position of tractor drive axle 
IC_y0 =   IC_y0f-L_0f*sin(IC_theta0);

%preparing the Grid points for forward simulation
[X,Y] = meshgrid(0:5:50,-50:5:50); 
gridpoints = [X(:), Y(:)];
szp=size(gridpoints,1);
figure
qinit=[0 0];
plot(qinit(1,1), qinit(1,2),'r*')
hold on
axis([-50 50 -50 50])
plot(gridpoints(:,1),gridpoints(:,2),'g.')
xlabel('x-position [m]');
ylabel('y-position [m]');
legend ('qinit','grid points');
if v_0==1
    title('Hmap for forward motion');
elseif v_0==-1
    title('Hmap for reverse motion');
end
%% 
%max num of time steps allowed or max time limit for individual forward
%simulation over a single reference line
indsimtime=300;
%a structure to store hmap data
H=struct;
H.inputgoalp=zeros(szp,2); 
H.trajobtained=zeros(indsimtime,4,szp);
H.cost=zeros(szp,1);
%is feasible when goal config reached or max time(max steps) limit reached.
%0 for feaible,1 for infeasible
H.feasibility=zeros(szp,1);
Simulink.Bus.createObject(H)
H.feasibility(11,1)=1; %presetting the (0,0) value as infeasible
%% tn tree with initial node at (0,0)
tn=zeros(1,12); 
tn(1,9:10)=[pi/2 pi/2];
% calling the sim model
 sim hmapsf
 %% 3d plot for Heuristic map 
run hmapsf_Plot
 
% %when intrested in plotting all the trajectories to each individual point
% %in the grid map uncomment this section and connect "H.trajobtained" to
% "out.traj" in ws() in the hmapsf.slx model, this connection increases
% simulation time

% figure
% for i = 1:szp
% tt=ans.traj.signals.values(:,:,i);
% plot(tt(:,1),tt(:,2))
% hold on
% end
% plot(50,-50,'r*')
% axis equal