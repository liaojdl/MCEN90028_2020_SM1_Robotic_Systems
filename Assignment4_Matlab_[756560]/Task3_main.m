% MCEN9028 Robotics System Assignment 4 task 3
% two sample trajectory using joint space PID controllers
% Version 1.0 2020 May, Jiawei Liao, 756560,
% mailto:liao2@student.unimelb.edu.au

%% Full clean up
clc; clear all; close all;

%% add in the values of parameters
m1 = 2; %(kg)
m2 = 1; %(kg)
Izz1 = 0.5;
Izz2 = 0.3;
Ixx1 = 0;
Iyy1 = 0;
Ixx2 = 0;
Iyy2 = 0;
L1 = 1;
L2 = 0.6;
rc1 = 0.5;
rc2 = 0.3;
g = 9.8;

% the PID and reference signal sampling time
dt_ref = 0.01;
% time of duration
tSeg = [5];


%% reference signal generation A
% conition of trajectory A
XrefA_i = [0.71;0];
XrefA_f = [1.485;0];
YrefA_i = [1.08;0];
YrefA_f = [0.041;0];
% polynomial coefficients
XrefA_Coeff = TrajGen01(XrefA_i,XrefA_f,tSeg);
YrefA_Coeff = TrajGen01(YrefA_i,YrefA_f,tSeg);
% value evaluated at time steps for trajectory of q1,q2
XrefA = TrajVal01(tSeg,XrefA_Coeff,dt_ref);
YrefA = TrajVal01(tSeg,YrefA_Coeff,dt_ref);
XdrefA = gradient(XrefA)./dt_ref;
YdrefA = gradient(YrefA)./dt_ref;

%% reference signal generation B
% conition of trajectory B
XrefB_i = [0.71;0];
XrefB_f = [1.36;0];
YrefB_i = [1.08;0];
YrefB_f = [-0.59;0];
% polynomial coefficients
XrefB_Coeff = TrajGen01(XrefB_i,XrefB_f,tSeg);
YrefB_Coeff = TrajGen01(YrefB_i,YrefB_f,tSeg);
% value evaluated at time steps for trajectory of q1,q2
XrefB = TrajVal01(tSeg,XrefB_Coeff,dt_ref);
YrefB = TrajVal01(tSeg,YrefB_Coeff,dt_ref);
XdrefB = gradient(XrefB)./dt_ref;
YdrefB = gradient(YrefB)./dt_ref;


%% robot workspace of link 1
xws = zeros(26,1);
yws = zeros(26,1);
j=2;
for i = (-60*pi/180):(5*pi/180):(60*pi/180)
    xws(j) = L1*cos(i);
    yws(j) = L1*sin(i);
    j = j+1;
end
sector_L1 = polyshape(xws,yws);

%% choose A or B to generate joint velocity reference for next parts
%if using A
[Qup,Qdown] = ik2d2link(XrefA(1),YrefA(1),L1,L2);
Q1ref = real(Qdown(1));
Q2ref = real(Qdown(2));
Xref = XrefA;
Yref = YrefA;
Xdref = XdrefA;
Ydref = YdrefA;

% if using B
% [Qup,Qdown] = ik2d2link(XrefB(1),YrefB(1),L1,L2);
% Q1ref = real(Qdown(1));
% Q2ref = real(Qdown(2));
% Xref = XrefB;
% Yref = YrefB;
% Xdref = XdrefB;
% Ydref = YdrefB;


%% Simulation and tuning for PID joint space controller for q1 and q2
dt = 0.01; 
tf = 5; 

% Initialise the robot to the initial position and velocity
q(1) = Q1ref(1);
q(2) = Q2ref(1);
qdot(1) = 0;
qdot(2) = 0;
j = 1;
theta = 1;
% initialise some PID parameters 
% the PID sampling time
dtPID = 0.001;
time = 0:dtPID:tf;
q1d_err_last = 0;
q2d_err_last = 0;
q1di_err = 0;
q2di_err = 0;
q1_last = Q1ref(1);
q2_last = Q2ref(1); 

for i = 1:length(time)
    % real time ik sol using Qdref = J^(-1)*Xdref
    if (rem(time(i),dt)==0)
        J = [- L2*sin(q(1) + q(2)) - L1*sin(q(1)), -L2*sin(q(1) + q(2)); ...
            L2*cos(q(1) + q(2)) + L1*cos(q(1)), L2*cos(q(1) + q(2))];
        qdref = J^(-1)*[Xdref(theta);Ydref(theta)];
        q1dref = qdref(1);
        q2dref = qdref(2);
        theta = theta+1;
    end
    
    % proportional reference error
    q1d_err_cur = q1dref - qdot(1);
    q2d_err_cur = q2dref - qdot(2);
    % the derivative of error using euler foward first order method
    q1d_err_d = (q1d_err_cur-q1d_err_last)/dtPID;
    q2d_err_d = (q2d_err_cur-q2d_err_last)/dtPID;
    % the integral of cumulative error, using rectangular method
    q1di_err = q1di_err+q1d_err_d*dtPID;
    q2di_err = q2di_err+q2d_err_d*dtPID;
    
    % P, proportional component
    % Kuq1 = 400, Puq1 = 0.4, kuq2 = 75, puq2 = 0.02
    Kp_q1 = 200;
    Kp_q2 = 37.5;
    tau_P = [Kp_q1*q1d_err_cur;Kp_q2*q2d_err_cur]; 
    % I, integral component
    Ki_q1 = 0.5;
    Ki_q2 = 0.1;
    tau_I = [Ki_q1*q1di_err;Ki_q2*q2di_err];
    % D, derivative component
    Kd_q1 = 0.125;  
    Kd_q2 = 0.005;
    tau_D = [Kd_q1*q1d_err_d;Kd_q2*q2d_err_d]; 
    
    % gravity compensation torque
    tau_G = [g*m2*(rc2*cos(q(1) + q(2)) + L1*cos(q(1))) + g*m1*rc1*cos(q(1)); g*m2*rc2*cos(q(1) + q(2))];
    % control torque should be the sum of the torque
    tau = tau_P+tau_I+tau_D+tau_G;
    if (rem(time(i),dt)==0)
        tau_actual = tau;
        tau1s(j) = tau(1);
        tau2s(j) = tau(2);
        j = j+1;
    end

    [t,y] = ode45(@(t,y) Task3_dynamics(t,y,tau_actual), [0, dtPID], [q(1), q(2), qdot(1), qdot(2)]);
    Ly = length(y(:,1));
    q(1) = y(Ly,1);
    q(2) = y(Ly,2);
    qdot(1) = y(Ly,3);
    qdot(2) = y(Ly,4);
    
    %%% Storing q and qdot values into an array
    q1s(i) = q(1);
    q2s(i) = q(2);
    qdot1s(i) = qdot(1);
    qdot2s(i) = qdot(2);

    
    % update last error 
    q1d_err_last = q1d_err_cur;
    q2d_err_last = q2d_err_cur;
end


%% response against reference
timep = 0:dt:tf;
figure
plot(timep,180*Q1ref/pi,'-',time,180*q1s/pi,'-')
grid on
grid minor
xlabel ('time (s)')
ylabel ('joint velocity (degree/s)')
title ('Reference vs Response')
legend('q_1 ref','q_1 actual')
figure
plot(timep,180*Q2ref/pi,'-',time,180*q2s/pi,'-')
grid on
grid minor
xlabel ('time (s)')
ylabel ('joint velocity (degree/s)')
title ('Reference vs Response')
legend('q_2 ref','q_2 actual')

%% q1d,q2d plot 
figure
plot(time,qdot1s*180/pi,'-o', time,qdot2s*180/pi,'-o')
grid on
grid minor
xlabel ('time (s)')
ylabel ('joint angle velocity (degree/s)')
title ('qd_1,qd_2 vs t')
legend('qd_1','qd_2')

timep = 0:dt:tf;
x_j2s = time;
y_j2s = time;

%% robot animation "screenshots"]
k = 0;
for i = 1:length(time)
    time_cur = time(i);
    x_j1 = L1*cos(q1s(i));
    y_j1 = L1*sin(q1s(i));
    x_j2 = L1*cos(q1s(i))+L2*cos(q1s(i)+q2s(i));
    y_j2 = L1*sin(q1s(i))+L2*sin(q1s(i)+q2s(i));
    x_j2s(i) = x_j2;
    y_j2s(i) = y_j2;
    if (rem(time(i),0.5)==0)
        
        % workspace of link2
        xwe = zeros(37,1);
        ywe = zeros(37,1);
        j=1;
        for theta = (-90*pi/180):(5*pi/180):(90*pi/180)
            xwe(j) = x_j1+L2*cos(theta+q1s(i));
            ywe(j) = y_j1+L2*sin(theta+q1s(i));
            j = j+1;
        end
        sector_L2 = polyshape(xwe,ywe);
        figure
        hold on
        plot(sector_L1,'edgecolor','none','FaceColor',[204,255,255]/255);
        plot(sector_L2,'edgecolor','none','FaceColor',[255,255,204]/255);
        view(0,90)
        grid on
        grid minor
        
        xlabel ('x(m)')
        ylabel ('y(m)')
        title(['Robot config at t=',num2str(time(i)),'s'])
        plot([0, x_j1], [0, y_j1],'Linewidth',5,'Color',[0, 0.4470, 0.7410]);
        plot([x_j1, x_j2], [y_j1, y_j2],'Linewidth',5,'Color',[0.8500, 0.3250, 0.0980]);
        plot(x_j2, y_j2,'o','MarkerSize',12.5,'MarkerFaceColor',[0.8500, 0.3250, 0.0980]);
 
        axis equal
        axis([-1.6 1.6 -1.6 1.6])
%         s=hgexport('readstyle','yyy');
%         fnam=['t3B_',num2str(k)]; % your file name
%         s.Format = 'eps'; %I needed this to make it work but maybe you wont.
%         hgexport(gcf,fnam,s);
        k = k+1;
    end
end

figure
plot(timep,Xref,'-',time,x_j2s,'-')
grid on 
grid minor
xlabel ('time(s)')
ylabel ('displacement(m)')
legend('x_{ref}(t)','x(t)')
title ('x(t) vs t (trajectory and reference)')

figure
plot(timep,Yref,'-',time,y_j2s,'-')
grid on 
grid minor
xlabel ('time(s)')
ylabel ('displacement(m)')
legend('y_{ref}(t)','y(t)')
title ('y(t) vs t (trajectory and reference)')


%% animating the robot as a .gif
%robot angle displacment over time
p1x = L1*cos(q1s);
p1y = L1*sin(q1s);
p2x = L2*cos(q1s + q2s)+ L1*cos(q1s);
p2y = L2*sin(q1s + q2s)+ L1*sin(q1s);

figure
hold on
axis equal
grid on
grid minor
axis([-1.6 1.6 -1.6 1.6])
h = animatedline('MaximumNumPoints',2,'Linewidth',5,'Color',[0, 0.4470, 0.7410]);
g = animatedline('MaximumNumPoints',2,'Linewidth',5,'Color',[0.8500, 0.3250, 0.0980]);
[A,map] = rgb2ind(frame2im(getframe),512);
imwrite(A,map,'task3B.gif','LoopCount',65535,'DelayTime',0.01);

for i=1:length(q1s)
    addpoints(h,0,0);
    addpoints(h,p1x(i),p1y(i));
    addpoints(g,p1x(i),p1y(i));
    addpoints(g,p2x(i),p2y(i));
    drawnow
    if(mod(i,25)==0)
        [A,map] = rgb2ind(frame2im(getframe),512);
        imwrite(A,map,'task3B.gif','WriteMode','append','DelayTime',0.05);
    end
end