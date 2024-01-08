clear,clc,close all
wall=[3,-3;3,1;5,1;5,3;-2,3;-2,0;-5,0;-5,-3;3,-3];

% P=[0,0;2,1.5;3.5,2;3,2.7;-.5,1.3;-2,-1;-3,-1;-3,-2;1,-2;2.5,1];
P=[0,0;0,1.5;1.5,2;1.5,-2;-2,-2];
% P=[sin(0:pi/10:2*pi);cos(0:pi/10:2*pi)]';

X=P(1,1);
Y=P(1,2);
th=0;
l_p=.14+2*.3*0;
X_l=X;
Y_l=Y+l_p/2;
X_r=X;
Y_r=Y-l_p/2;

i=1;
d=sqrt((X(end)-P(i+1,1))^2+(Y(end)-P(i+1,2))^2);
th_des=atan2(P(i+1,2)-Y(end),P(i+1,1)-X(end));
th_des_past=th_des;
d_min=.01;
l=.1;
v=l/9;
dt=.005;
r=.4;
d_th_max=v/r*dt;
figure
hold on
axis equal
counter=0;
while(d>d_min)
    counter=counter+1;
     th_des=atan2(P(i+1,2)-Y(end),P(i+1,1)-X(end));
     while th_des-th_des_past>pi
         th_des=th_des-pi*2;
     end
     while th_des-th_des_past<-pi
         th_des=th_des+pi*2;
     end
     th_des_past=th_des;
%     th_des=atan((P(i+1,2)-Y(end))/(P(i+1,1)-X(end)));
dth=th_des-th(end);
if dth<-d_th_max
    dth=-d_th_max;
elseif dth>d_th_max
    dth=d_th_max;
end
th(end+1)=th(end)+dth;
X(end+1)=X(end)+v*cos(th(end))*dt;
Y(end+1)=Y(end)+v*sin(th(end))*dt;

X_l(end+1)=X(end)-l_p/2*sin(th(end));
Y_l(end+1)=Y(end)+l_p/2*cos(th(end));
X_r(end+1)=X(end)+l_p/2*sin(th(end));
Y_r(end+1)=Y(end)-l_p/2*cos(th(end));



   d=sqrt((X(end)-P(i+1,1))^2+(Y(end)-P(i+1,2))^2);
   if d<d_min
    i=i+1;
    if i>size(P,1)-1
       break
   end
    d=sqrt((X(end)-P(i+1,1))^2+(Y(end)-P(i+1,2))^2);
   end
   
%    if(mod(counter,3000)==0)
%    clf
%    hold on
%    plot(wall(:,1),wall(:,2),'k')
%    plot(X,Y)
%    plot(X_l,Y_l)
%    plot(X_r,Y_r)
%    scatter(P(:,1),P(:,2))
%    axis equal
%    pause(dt/100)
%    end
end



%%

figure
hold on
axis equal
plot(wall(:,1),wall(:,2),'k')
   plot(X,Y)
   plot(X_l,Y_l)
   plot(X_r,Y_r)

scatter(P(:,1),P(:,2))
m=l/v/dt;

steps_l=[0,X_l(m:2*m:end),X_l(end);l_p/2,Y_l(m:2*m:end),Y_l(end);0,th(m:2*m:end),th(end)];
steps_r=[0,X_r(2*m:2*m:end),X_r(end);-l_p/2,Y_r(2*m:2*m:end),Y_r(end);0,th(2*m:2*m:end),th(end)];


scatter(steps_l(1,:),steps_l(2,:))
scatter(steps_r(1,:),steps_r(2,:))





