 clear
%Order of the files that have to be run individually
%1.clrrtsf_Initializ.m
%2.clrrtsf.slx
%3.clrrtsolutionplot.m
%the current file, clrrtsf_Initializ.m has to be run before running clrrtsf.slx
%this file initiates the hmaps, max Number of nodes, Obstacle space,
% "clrrtsolutionplot.m" will display number of solutions found,
% 1.plots the tree expansion
% 2.if any solutions found, it finds the optimal path(shortest curve length)
% and plots the solution path and its respective pice wise linear reference
% path used as aref path by SAV to reach goal region from start region
%if no solution found "clrrtsolutionplot.m" throws error
 

%using Hmap data obtained by running hmapsf_Initializ.m, the data is stored
%in heuristicmap.mat with names Hmap_frwd and Hmap_rev, these arrays will
%be used to build a look-up table in simulink.

%older hmap
% load('heuristicmap.mat', 'hmapfrwd', 'hmaprev')
% x=hmapfrwd(:,1);
% y=hmapfrwd(:,2);
% znew=hmapfrwd(:,5);

add path lib
%for the look-up table go to chart>f_()>ForIteratorsubsystem>subsystem
load('heuristicmap.mat', 'Hmap_frwd', 'Hmap_rev')

%interpolating existing values which are at resolution of 5 meters
[xnewt,ynewt] = ndgrid(-50:0.5:50,-50:0.5:50);

%forward hmap look-uptable
x=Hmap_frwd(:,1);
y=Hmap_frwd(:,2);
znew=Hmap_frwd(:,3);


Zf = griddata(x,y,znew,xnewt,ynewt);
xx=[-50:0.5:50]';
yy=[-50:0.5:50]';

%reverse hmap look-uptable

%old version
% znewr=hmaprev(:,5);
% x=hmaprev(:,1);
% y=hmaprev(:,2);

x=Hmap_rev(:,1);
y=Hmap_rev(:,2);
znewr=Hmap_rev(:,3);

Zr = griddata(x,y,znewr,xnewt,ynewt);
xx=[-50:0.5:50]';
yy=[-50:0.5:50]';


%obstacle map
%% The DPD scenario.
% Creating obstacles
% We have to define the vertices which when joined forms the obstacle. 
obsc= 5; % Number of obstacles
% Please put the obstacle with most number of vertices as the first obstacle. 
obsx(1,:) = [28,58.6,58.6,61.4,61.4,108,108,200.5,200.5,204.25,204.25,228,228,285,285,228,228,111.4,111.4,108.6,108.6,108,108,91.4,91.4,88.6,88.6,28];
obsy(1,:) = [53,53,69.5,69.5,53,53,61,61,77.5,77.5,61,61,57,57,140,140,136,136,119.5,119.5,136,136,144,144,127.5,127.5,144,144];
% Adding 2nd obstacle
temp1   =   zeros(1,length(obsx(1,:)));                                     % Matching the number of elements
ind     =   find([72,240,255,200,188.3327,186.352,197.2,87,72]);            % Assigning the indexes     
temp1(ind)  =   [72,240,255,200,188.3327,186.352,196.4,87,72];              % Putting the obstacle coordinates
obsx(2,:)   =   temp1;                                                      % Putting in the obstacle matrix. 

temp1   =   zeros(1,length(obsy(1,:)));
ind     =   find([13,13,33,33,21.33,18.94,33,33,13]);
temp1(ind)  =   [13,13,33,33,21.33,23.3099,33,33,13];
obsy(2,:)   =   temp1;

% Adding 3rd obstacle
temp1   =   zeros(1,length(obsx(1,:)));
ind     =   find([27,243,260,44]);
temp1(ind)  =   [27,243,260,44];
obsx(3,:)   =   temp1;

temp1   =   zeros(1,length(obsy(1,:)));
ind     =   find([168,168,178,178]);
temp1(ind)  =   [168,168,178,178];
obsy(3,:)   =   temp1;
% Adding 4rd obstacle
temp1   =   zeros(1,length(obsx(1,:)));
ind     =   find([100,150,150,100]);
temp1(ind)  =   [100,150,150,100];
obsx(4,:)   =   temp1;

temp1   =   zeros(1,length(obsy(1,:)));
ind     =   find([195,195,192.5,192.5]);
temp1(ind)  =   [195,195,190,190];
obsy(4,:)   =   temp1;
% Adding 5rd obstacle
temp1   =   zeros(1,length(obsx(1,:)));
ind     =   find([162.6,180,180,162.6]);
temp1(ind)  =   [162.6,180,180,162.6]+20;
obsx(5,:)   =   temp1;

temp1   =   zeros(1,length(obsy(1,:)));
ind     =   find(([195,195,192.5,192.5]));
temp1(ind)  =   [195,195,190,190];
obsy(5,:)   =   temp1;

%bringing obsx and obsy into one 3d array 'obs'
obs=zeros(obsc,length(obsx(1,:)),2);
obs(:,:,1)=obsx;
obs(:,:,2)=obsy;
% % If there are more obstacles add them in this fashion and update the 
% % obstacle count.

%If you want to visualize the path planning process live plese uncomment
%the following lines.(WARNING: INCREASES COMPUTATIONAL TIME)
figure(1);
fill([0,286,286,0],[0,0,200,200],'c');
hold on;
for i=1:obsc
    obsxtemp=obsx(i,:);
    obsytemp=obsy(i,:);
    obsxtemp=obsxtemp(obsxtemp~=0);
    obsytemp=obsytemp(obsytemp~=0);
    fill(obsxtemp,obsytemp,'r');
end

%%

LD=10; %lookahead circle radius
% pgoal=[170 193 pi]; pinit=[190 187 pi];
%pgoal=[202.4 71.5 (3*pi)/2]; pinit=[230 50 0];
pgoal=[90 133.5 (pi)/2]; pinit=[80 160 pi 0];
hold on
plot(pgoal(1,1), pgoal(1,2),  'g*','linewidth',3');
plot(pinit(1,1), pinit(1,2),  'g*','linewidth',3');
indsimtime=150; % individual simulation time
numofnodes=500; %total num of nodes we let the tree to expand
N=numofnodes;
tn=zeros(N,14);

%as inbuilt random number genrating function is consuming time,an array 
%filled with random numbers is used to simulate the random number
%generation within the clrrt planner to increase the simulation speed. 
load('randombum.mat', 'randnum');
%   randnum=rand(10^6,1);
%  save('randombum.mat');
% save('randombum.mat','randnum','-append');

%details about the data collected in the tree tn (Nx14)
%column details-1,2-Node co-ordinates;3-node number;4-parent node num;
%5samplingtype(goalsampling-0,randomsampling-1); 6-cost-arclength between
%current node to previous node; 7-cumilative cost-total arc length from
%current node to the root node; 8-direction of motion (forward-1 or
%reverse-0); 9-orientation of the tractor; 10-orientation of the semi
%trailer; 11-feasibility; 12-number of time steps taken for that
%particular extension; 13,14 Co-ordinates of selected sampled points that 
%enter tree as nodes
tn(1,3)=1;%first node num
sztn=0;

%vehiclepaths
%1-xcod, 2-ycod, 3-tracyawangle theta_0, 4-semitraileryawangletheta_1,
%5-indicatorofgoalreached 6-indicatorof max timesteps reached
veh_path_col=zeros(indsimtime,6,N);%vehicle paths collection;
veh_path=zeros(indsimtime,6);

%  % Initialize vehicle model
v_0 =  0;  
L_0f = 3.8;  %(tractor wheelbase)
L_1f = 7.21; %(trailer wheel base)
L_0b = 0.48;  %(distance tractor rear axle-coupling point)


    