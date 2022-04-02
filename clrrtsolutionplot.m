%Order of the files that have to be run individually for clrrt planner
%1.clrrtsf_Initializ.m
%2.clrrtsf.slx
%3.clrrtsolutionplot.m

% "clrrtsolutionplot.m" will display number of solutions found,
% 1.plots the tree expansion
% 2.if any solutions found, it finds the optimal path(shortest curve length)
% and plots the solution path and its respective pice wise linear reference
% path used as aref path by SAV to reach goal region from start region
%if no solution found "clrrtsolutionplot.m" throws error

%% plot the tree exapnsion
figure(1)
plot(pgoal(1,1),pgoal(1,2),'r*');
hold on
for i=2:N
x =find(out.veh_path_col(:,1,i));
y=find(out.veh_path_col(:,2,i));
z=union(x,y);
if out.tree(i,8)==-1
plot(out.veh_path_col(z(1:end-1,1),1,i),out.veh_path_col(z(1:end-1,1),2,i),'b','LineWidth',1.5);
hold on;
else
plot(out.veh_path_col(z(1:end-1,1),1,i),out.veh_path_col(z(1:end-1,1),2,i),'r','LineWidth',1.5);
hold on;
end
end
%% 

%obtaining nodes that hit goal region
sznodes=size(tn,1);
tn=out.tree;
collection=0;
j=0;
for i=1:sznodes
d=sqrt((tn(i,1)-pgoal(1,1))^2+(tn(i,2)-pgoal(1,2))^2);
e=abs(pgoal(1,3)-wrapTo2Pi(tn(i,9)));
D(i)=d;
E(i)=e;
if (d)<=1 && e <0.5
j=j+1;
collection(j,1)=i;
collection(j,2)=d;
collection(j,3)=e;
end
end
disp('number of solutions obtained = ');
disp(j);

%obtaining respective trail nodes(starting from goal region hit node to initial node) 
szcol=size(collection,1);
for b_=1:szcol
i=collection(b_,1);
childnoden=tn(i,3);
childnodepn=tn(i,4);
solpathnodes=0;
k=0;
while true
k=k+1;
xx1_(b_).solpathnodes(k,1)=tn(childnoden,3);
b=childnodepn;
parentnoden=tn(b,3);
parentnodepn=tn(b,4);
%current parentnode will be next steps childnode
childnoden=parentnoden;
childnodepn=parentnodepn;
if b==1
    xx1_(b_).solpathnodes=flip(xx1_(b_).solpathnodes);
for ii=1:k
%obtaining the trajectory for the solution path
xx1_(b_).solpathtraj(1:indsimtime,:,ii)=out.veh_path_col(:,1:4,xx1_(b_).solpathnodes(ii,1));
end
break
end
end
%assigning cumulative cost for that particular solution path
xx1_(b_).solpathcost=tn(i,7);
end

 
%% 



%optimal -cheapest path
%finding cheapest path based on the cumulative cost calculated from the
%solution path curve length
for j=1:szcol
costarray(j,1)=xx1_(j).solpathcost;
end
%% sorting mbased on the minimum cost

  [val, idx] = min(costarray);
%% code snippet in this section is used to visualize the SAV motion from 
%earlier saved solution paths
%   load('solcollection.mat', 's1')
% % load('solcollection.mat', 's3')
%    idx=1;
%    xx1_=s1;
%% plotting optimal path
szsolpathnodes=size(xx1_(idx).solpathnodes,1);
figure(1)
id=0;
for m=1:szsolpathnodes
hold on
lidx=find(xx1_(idx).solpathtraj(:,1,m),1,'last');
lidy=find(xx1_(idx).solpathtraj(:,2,m),1,'last');
lid=union(lidx,lidy);
xp(id+1:id+lid,1)=xx1_(idx).solpathtraj(1:lid,1,m);
yp(id+1:id+lid,1)=xx1_(idx).solpathtraj(1:lid,2,m);
thetap(id+1:id+lid,1)=xx1_(idx).solpathtraj(1:lid,3,m);
gammap(id+1:id+lid,1)=xx1_(idx).solpathtraj(1:lid,4,m);
id=id+lid;
 plot(xx1_(idx).solpathtraj(:,1,m),xx1_(idx).solpathtraj(:,2,m),'b.'  );
end

%truck movement% Calculation for the motion of the truck%%
L_0f = 3.8;  %(tractor wheelbase)
L_1f = 7.21; %(trailer wheel base)
L_0b = 0.48;  %% Distance of 1st king-pin to tractor drive axle [m][m]
oh_1b = 4.3260;  % Longitudinal distance from the trailer axle to the end of the trailer [m] (60% of the WB)
oh_1f = L_1f+1;  % Longitudinal distance from the trailer axle to the front of the trailer [m]
w_1   = 2.5;     % Width of a trailer [m]
% Now  we will create vectors to each corner of the trailer
% Length of vector 1, 2 is the same and 3,4 is the same
lv12_1 = hypot(oh_1b,(w_1/2)); % Length of Vector 1 and 2
lv34_1 = hypot((w_1/2),oh_1f); % Length of Vector 3 and 4

% Now we have to find the corners of the tractor. The tractor and the
% trailer have one point the same and fixed which is the king pin.

oh_0f = 1.5; % Frontal foverhang of the truck [m
oh_0b = 0.94; % Distance from the drive axle to the end of the tractor [m]
lv12_0 = hypot((oh_0b),(w_1/2)); % Length of Vector 1 and 2
lv34_0 = hypot((w_1/2),(L_0f+oh_0f)); % Length of Vector 3 and 4

% For Trailer
% Angle of the vectors
av1 = (180/pi*thetap)+90+atand(oh_1b/(w_1/2)); % Angle of Vector 1
av2 = (180/pi*thetap)-90-atand(oh_1b/(w_1/2)); % Angle of Vector 2
av3 = (180/pi*thetap)-atand((w_1/2)/oh_1f); % Angle of Vector 3
av4 = (180/pi*thetap)+atand((w_1/2)/oh_1f); % Angle of Vector 4

% Finding the corner points of the trailer at every (xp,yp)
xv1_1 = xp+lv12_1*cosd(av1);
yv1_1 = yp+lv12_1*sind(av1);
xv2_1 = xp+lv12_1*cosd(av2);
yv2_1 = yp+lv12_1*sind(av2);
xv3_1 = xp+lv34_1*cosd(av3);
yv3_1 = yp+lv34_1*sind(av3);
xv4_1 = xp+lv34_1*cosd(av4);
yv4_1 = yp+lv34_1*sind(av4);

%For tractor
x_1f = xp+L_1f*cos(thetap); % Position of the king pin
y_1f = yp+L_1f*sin(thetap);
theta_0 = thetap+gammap; % Orientation angle of tractor [rad]
x_0 = x_1f-L_0b*cos(theta_0); % Position of center of the driven axle
y_0 = y_1f-L_0b*sin(theta_0);
% Angle of the vectors
av1 = (180/pi*(theta_0))+90+atand(oh_0b/(w_1/2)); % Angle of Vector 1
av2 = (180/pi*(theta_0))-90-atand(oh_0b/(w_1/2)); % Angle of Vector 2
av3 = (180/pi*(theta_0))-atand((w_1/2)/(oh_0f+L_0f)); % Angle of Vector 3
av4 = (180/pi*(theta_0))+atand((w_1/2)/(oh_0f+L_0f)); % Angle of Vector 4

% Finding the corner points of tractor at every (xp,yp)
xv1_0 = x_0+lv12_0*cosd(av1);
yv1_0 = y_0+lv12_0*sind(av1);
xv2_0 = x_0+lv12_0*cosd(av2);
yv2_0 = y_0+lv12_0*sind(av2);
xv3_0 = x_0+lv34_0*cosd(av3);
yv3_0 = y_0+lv34_0*sind(av3);
xv4_0 = x_0+lv34_0*cosd(av4);
yv4_0 = y_0+lv34_0*sind(av4);
xc=[];
yc=[];
    xc=[[xv1_1 xv2_1 xv3_1 xv4_1 xv1_0 xv2_0 xv3_0 xv4_0];xc];
    yc=[[yv1_1 yv2_1 yv3_1 yv4_1 yv1_0 yv2_0 yv3_0 yv4_0];yc];

  for i=1:(length(xc(:,1))/5):length(xc(:,1))%%
        trailer= polyshape([xc(i,1);xc(i,2);xc(i,3);xc(i,4)],[yc(i,1);yc(i,2);yc(i,3);yc(i,4)]);
        tractor= polyshape([xc(i,5);xc(i,6);xc(i,7);xc(i,8)],[yc(i,5);yc(i,6);yc(i,7);yc(i,8)]);
        figure(1);
        axis equal;
       f= plot(tractor);
      ff=  plot(trailer);
%         delete(f);
  %      delete(ff);
        drawnow limitrate;
        
  end
%    plot transparent filled circles
hold on
plot(pgoal(1,1), pgoal(1,2),  'g*','linewidth',3');
x = [pgoal(1,1) pinit(1,1)];
y1 = [pgoal(1,2) pinit(1,2)];
a1 = 10*[30 30];
scatter(x,y1,a1,'MarkerFaceColor','k','MarkerEdgeColor','k',...
'MarkerFaceAlpha',.2,'MarkerEdgeAlpha',.2)
axis([-40 40 -40 40])
% %textbox legend
text(100, 160, {'{\color{black} ---- } Fwd motion', '{\color{magenta} ---- } Rev motion', '{\color{yellow} ---- } piece wise linear reference path'},'fontsize',15,'EdgeColor', 'k');
% title(strcat('X_i=',num2str(i_state.x),', Y_i=',num2str(i_state.y),', \theta_i=',num2str(i_state.t),', X_f=',num2str(f_state.x),', Y_f=',num2str(f_state.y),', \theta_f=',num2str(f_state.t),', Comp Time=',num2str(toctime)));

title(strcat('Initial pose=[',num2str(pinit),'], Goal pose=[',num2str(pgoal),'], \Delta r in m =',num2str(0.6),', \Delta \theta in deg=',num2str(0.05),', Num of nodes=',num2str(N)));

xlabel('x-position [m]','fontsize',22);
ylabel('y-position [m]','fontsize',22);
xlim([0 285]);
ylim([0 200]);

%% to obtain the pice wise linear reference paths that drive the SAV 
%from initial pose to final pose

refpath_nodenum=xx1_(idx).solpathnodes;
refpath_nodeco_ord(1,1:2)=pinit(1,1:2);
for b=1:szsolpathnodes
    refpath_nodeco_ord(b+1,1:2)=[tn(refpath_nodenum(b,1),13:14)];
    
end

co_ord=refpath_nodeco_ord;
figure(1)
hold on
plot(co_ord(:,1),co_ord(:,2),'y*','LineWidth',5);
hold on
plot(co_ord(:,1),co_ord(:,2),'y','LineWidth',2);
%solution path
figure(1)
hold on
plot(xp(:,1),yp(:,1),'m.','LineWidth',2);



