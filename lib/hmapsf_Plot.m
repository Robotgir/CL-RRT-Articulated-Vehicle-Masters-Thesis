out=ans;
szpgp=size(gridpoints,1);

if v_0 >0
    %collecting the hmap data to an array for 3d plotting
hmapfrwd=zeros(szpgp,5);
hmapfrwd(:,1:2)=gridpoints(:,1:2);
hmapfrwd(:,3)=out.Hcost.signals.values(:,1);
hmapfrwd(:,4)=out.feasibility.signals.values(:,1);
hmapfrwd(:,5)=hmapfrwd(:,3);
%mirroring obtained results to other half on x axis, i.e from -50 to 0,
%repeated co-ordinate values are removed and a 3d surp plot is created with
%co-ordinates on x,y axis cost on z axis
rsha=reshape(hmapfrwd(:,3),21,11);
Z(:,1:21)=[fliplr(rsha)  rsha(:,2:11)];
[X,Y] = meshgrid(-50:5:50,-50:5:50);
[xnew,ynew] = meshgrid(-50:0.35:50,-50:0.35:50);
Znew = interp2(X,Y,Z,xnew,ynew,'cubic',0);
figure
surf(xnew,ynew,Znew)
colorbar
title('Heuristics forward simulation');
xlabel('x in m');
ylabel('y in m');
brighten(parula,-0.5)
Hmap_frwd=[X(:) Y(:) Z(:)];
save('heuristicmap.mat','Hmap_frwd','-append');
elseif v_0<0
        %collecting the hmap data to an array for 3d plotting
hmaprev=zeros(szpgp,5);
hmaprev(:,1:2)=gridpoints(:,1:2);
hmaprev(:,3)=out.Hcost.signals.values(:,1);
hmaprev(:,4)=out.feasibility.signals.values(:,1);
hmaprev(:,5)=hmaprev(:,3);
%mirroring obtained results to other half on x axis, i.e from -50 to 0,
%repeated co-ordinate values are removed and a 3d surp plot is created with
%co-ordinates on x,y axis cost on z axis
rsha=reshape(hmaprev(:,3),21,11);
Z(:,1:21)=[fliplr(rsha)  rsha(:,2:11)];
[X,Y] = meshgrid(-50:5:50,-50:5:50);
[xnew,ynew] = meshgrid(-50:0.5:50,-50:0.5:50);
Znew = interp2(X,Y,Z,xnew,ynew,'cubic',0);
figure
surf(xnew,ynew,Znew)
colorbar
title('Heuristics reverse simulation');
xlabel('x in m');
ylabel('y in m');
brighten(parula,-0.5)
Hmap_rev=[X(:) Y(:) Z(:)];
save('heuristicmap.mat','Hmap_rev','-append');
end
shading interp;
