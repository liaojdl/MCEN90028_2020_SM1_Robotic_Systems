function success = plotxyt_xvyvt_xvyv(Xi,Xf,Yi,Yf,xt,yt,xvt,yvt,tSeg,step,axlim1,axlim2,axlim3)
%  author: Jiawei Liao 756560 liao2@student.unimelb.edu.au
%  version: Apr 2020
%% description
%  plot the x and y displacement and velocity over time for a trajectory 
%  of several segments
%% input
%  &param Xi,Yi, is the ((P+1)/2) X kmax matrix containing the initial 
%               conditions for all the segments, for example, for cubic 
%               polynomials (P = 3), init will contain the initial 
%               position, initial velocity (therefore, 
%               ((P + 1)/2) = ((3 + 1)/2) = 2 for all kmax segments),
%               therefore init will be a 2 by kmax matrix.
%  &param Xf,Yf, final is the ((P + 1)/2)  kmax matrix containing the 
%               final conditions for all the segments.
%  &param xt,yt, x(t) , y(t)
%  &param xvt,yvt, xv(t) , yv(t)
%  &param tseg, is an array of size kmax X 1, containing the time duration 
%               of each segment of the trajectory.
%  &param step, plot resolution
%  &param axlim1, displacement plot axes limit
%  &param axlim2, velocity plot axes limit
%  &param axlim3, trajectory axes limit
%% output
%  &param xt,yt,xvt,yvt, x,y displacement, x,y velocity

%% some computations 
% total time
tTotal = sum(tSeg);
% key points
xk = [Xi(1,:),Xf(1,end)];
xvk = [Xi(2,:),Xf(2,end)];
yk = [Yi(1,:),Yf(1,end)];
yvk = [Yi(2,:),Yf(2,end)];

%% plot
figure
grid on
grid minor
hold on
ts = 0:step:tTotal;
xp = plot(ts,xt,'Color',[0, 0.4470, 0.7410]);
yp = plot(ts,yt,'Color',[0.8500, 0.3250, 0.0980]);
axis equal
axis(axlim1)

% axis equal
% xlim([0 8])
% ylim([0 20])
xlabel("time(s)")
ylabel("displacement(m)")
legend([xp,yp],"x(t)","y(t)");
keyxplot = plot(cumsum([0;tSeg]),xk,'ko');
keyyplot = plot(cumsum([0;tSeg]),yk,'ko');
set(get(get(keyxplot,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
set(get(get(keyyplot,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
title("x(t) and y(t) vs t")

% velocity plot
figure
hold on
grid on
grid minor
xvp = plot(ts,xvt,'Color',[0, 0.4470, 0.7410]);
yvp = plot(ts,yvt,'Color',[0.8500, 0.3250, 0.0980]);
axis equal
axis(axlim2)

% axis equal
% xlim([0 8])
% ylim([-1 3.5])
xlabel("time(s)")
ylabel("velocity(m/s)")
legend([xvp,yvp],"xv(t)","yv(t)");
keyxplot = plot(cumsum([0;tSeg]),xvk,'ko');
keyyplot = plot(cumsum([0;tSeg]),yvk,'ko');
set(get(get(keyxplot,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
set(get(get(keyyplot,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
title("xv(t) and yv(t) vs t")


% trajectory plot
figure
grid on
grid minor
hold on
plot(xt,yt);
plot(xk,yk,'ko');
axis equal
axis(axlim3)
title("xy trajectory")
xlabel("x displacement(m)")
ylabel("y displacement(m)")

success = 1;
end