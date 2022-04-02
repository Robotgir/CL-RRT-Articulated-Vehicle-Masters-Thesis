clear
clc
%FOR piece wise linear line segements please enter waypoints co-ordinates 
wayp=[0 0;0 20;20 20;20 -40; -40 -40; 0 -80];%];%
%please enter -1 for reverse motion and +1 for forward motion
v_0 =  -1;                          %tractor velocity in m/s
stept=0.01;

%.........................................................................
szwp=size(wayp,1);                 %num of waypoints
if v_0>0
IC_theta1=(pi)/2;                  %enter initial semitrailer orientation for forward direction
gamma_1=0;                         %enter initial articulation angle
elseif v_0<0
IC_theta1=(3*pi)/2;                %enter initial semitrailer orientation  for reverse direction
gamma_1=0;                         %enter initial articulation angle    
end

%Initializing vehicle model
L_0f = 3.8;                        %(tractor wheelbase)
L_1f = 7.21;                       %(trailer wheel base)
L_0b = 0.48;                       %(distance tractor rear axle-coupling point)

LD=15;                             %look ahead circle radius in m

 %updating the vehicle position initial conditions to controller
IC_x1= 0;                                   % initial x-position of semi axle 
IC_y1 = 0;
IC_theta0 =gamma_1+IC_theta1;               % initial yaw angle of tractor [deg]


IC_x1f= L_1f*cos(IC_theta1)+IC_x1 ;         
IC_y1f =L_1f*sin(IC_theta1)+IC_y1 ;

IC_x0f= (L_0f-L_0b)*cos(IC_theta0)+IC_x1f ;                                                         % initial x-position of semi axle 
IC_y0f =(L_0f-L_0b)*sin(IC_theta0)+IC_y1f ;

IC_x0 =   IC_x0f-L_0f*cos(IC_theta0);       % initial x-position of tractor drive axle 
IC_y0 =   IC_y0f-L_0f*sin(IC_theta0);
%% 
       
sim steercont_SAVIK_2019b_
out=ans;              
 %% 
%plotting 

plot_data_C(out,wayp,v_0,LD);               