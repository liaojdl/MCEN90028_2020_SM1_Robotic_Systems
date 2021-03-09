% MCEN9028 Robotics System Assignment 4 task 5
% implementing reactive obstacle avoidance
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
pA = [0.71,1.08];
pB = [1.485,0.041];
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

[Qup,Qdown] = ik2d2link(XrefA(1),YrefA(1),L1,L2);
Q1ref = real(Qdown(1));
Q2ref = real(Qdown(2));
Xref = XrefA;
Yref = YrefA;
Xdref = XdrefA;
Ydref = YdrefA;


%% Simulation and tuning for PID joint space controller for q1 and q2
dt = 0.01; 
tf = 5; 

% Initialise the robot to the initial position and velocity
q(1) = Q1ref(1);
q(2) = Q2ref(1);
qdot(1) = 0;
qdot(2) = 0;
j = 1;
k = 1;

% initialise some PID parameters 
% the PID sampling time
dtPID = 0.001;
time = 0:dtPID:tf;
q1d_err_last = 0;
q2d_err_last = 0;
q1di_err = 0;
q2di_err = 0;
% init error terms
xE = Xref(1);
yE = Yref(1);
exi_traj = 0;
eyi_traj = 0;

%initialise obstacle
[do,tho] = sensor_t5(1,0,0);
dOOS = do;
warn_flag = 0;
evasive_flag = 1;

%min clearance
q2lim = 90*pi/180;
MIN_dis = sqrt((L1+cos(q2lim))^2+(L2*sin(q2lim))^2);
n = 1;

% main loop
for i = 1:length(time)
    
    % real time ik sol using Qdref = J^(-1)*Xdref
    if (rem(time(i),dt)==0)
        
        if (evasive_flag == 0)
            if (~isnan(pDx))
                time_left = 5-time(i);
                disAV = sqrt((pDx(1)-pA(1))^2+(pDx(2)-pA(2))^2);
                disVB = sqrt((pDx(1)-pB(1))^2+(pDx(2)-pB(2))^2);
                t_evade = disAV/(disAV+disVB);
                t_evade = round(t_evade,2);
                tSeg = [t_evade,time_left-t_evade];
                % conition of trajectory A
                XrefA_i = [Xref(k),pDx(1);xdE,0];
                YrefA_i = [Yref(k),pDx(2);ydE,0];
                XrefA_f = [pDx(1),1.485;0,0];
                YrefA_f = [pDx(2),0.041;0,0];
                % polynomial coefficients
                XrefA_Coeff = TrajGen01(XrefA_i,XrefA_f,tSeg);
                YrefA_Coeff = TrajGen01(YrefA_i,YrefA_f,tSeg);
                Xrefnew = TrajVal01(tSeg,XrefA_Coeff,dt_ref);
                Yrefnew = TrajVal01(tSeg,YrefA_Coeff,dt_ref);
                Xref(k:end) = Xrefnew;
                Yref(k:end) = Yrefnew;
                Xdref(k:end) = gradient(Xrefnew)./dt_ref;
                Ydref(k:end) = gradient(Yrefnew)./dt_ref;
            end
            evasive_flag = 1;
        end
        
        J = [- L2*sin(q(1) + q(2)) - L1*sin(q(1)), -L2*sin(q(1) + q(2)); ...
            L2*cos(q(1) + q(2)) + L1*cos(q(1)), L2*cos(q(1) + q(2))];
        % trajectory error proportional
        ex_traj = Xref(k)-xE;
        ey_traj = Yref(k)-yE;
        % proportional gain
        Kpx = 5;
        Kpy = 5;
        % trajectory error integral
        exi_traj = exi_traj+ex_traj*dt;
        eyi_traj = eyi_traj+ey_traj*dt;
        % integral gain
        Kix = 1;
        Kiy = 1;
        % a PI controller to update the task space velocity reference
        xdref_cur = Xdref(k) + Kpx*ex_traj + Kix*exi_traj;
        ydref_cur = Ydref(k) + Kpy*ey_traj + Kiy*eyi_traj;

        % compute joint space velocity ref for robot
        qdref = J^(-1)*[xdref_cur;ydref_cur];
        q1dref = qdref(1);
        q2dref = qdref(2);
        if (warn_flag == 1)
            q1dref = 0;
            q2dref = 0;
        end        
        
        % stop if too close to obstacle, 
        if (do<=0.005)
            q2dref = 0;
            q1dref = 0;
        end
        % stop if collision is inevitable given the obstacle distance is 
        % within the minimum the robot can manuovere
        if (dOOS<(MIN_dis))
            if (warn_flag==0)
                fprintf("No possible way to avoid collision.\n")
                warn_flag = 1;
            end
        end
        k = k+1;
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
    [t,y] = ode45(@(t,y) Task5_dynamics(t,y,tau_actual), [0, dtPID], [q(1), q(2), qdot(1), qdot(2)]);
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
    
    % the end effector position
    xE = L1*cos(q1s(i))+L2*cos(q1s(i)+q2s(i));
    yE = L1*sin(q1s(i))+L2*sin(q1s(i)+q2s(i));
    
    
    J = [- L2*sin(q(1) + q(2)) - L1*sin(q(1)), -L2*sin(q(1) + q(2)); ...
            L2*cos(q(1) + q(2)) + L1*cos(q(1)), L2*cos(q(1) + q(2))];
    % the end effector velocity
    XdE = J*[qdot(1);qdot(2)];
    xdE = XdE(1);
    ydE = XdE(2);
    
    % check the obstacle.
    [do,tho] = sensor_t5(0,xE,yE);
    dos(i) = do;
    thos(i) = tho;
    
    % use the above information to do a mapping of the obstacle
    xOs(i) = xE+do*cos(tho);
    yOs(i) = yE+do*sin(tho);
    
     if (rem(time(i),dt)==0)
        dOOS = sqrt((xE+do*cos(tho))^2+(yE+do*sin(tho))^2);
        if (do<0.35 && do>0)
            xcr(n) = xOs(i);
            ycr(n) = yOs(i);
            n = n+1;
        end
        
        % uses 50 points 0.5 seconds to map the circle
        if (n==50)
            [xcObs,ycObs,RObs,EqObs] = circfit(xcr,ycr);
            [pD1,pD2] = findviapoint([0.71,1.08],[1.485,0.041],[xcObs,ycObs],RObs+0.025);
            if pD1(2)<pD2(2)
                pDx = pD1;
            else
                pDx = pD2;
            end
            left_most_x = xcObs-RObs;
            if pDx(1) > left_most_x
                pDx(1) = left_most_x-0.05;
            end
            evasive_flag = 0;
        end   
     end
end



timep = 0:dt:tf;
x_j2s = time;
y_j2s = time;

%% robot animation "screenshots"]
k = 0;
figure
hold on
% detected points of obstacle
plot(xOs,yOs,'kx','MarkerSize',2);
% mapping of obstacle circle using fitting tools
angle = 0:pi/180:2*pi;
xOs_map = RObs*cos(angle)+xcObs;
yOs_map = RObs*sin(angle)+ycObs;
plot(xOs_map,yOs_map,'m-');
title('Robot config every 0.5s')
for i = 1:length(time)
    time_cur = time(i);
    x_j1 = L1*cos(q1s(i));
    y_j1 = L1*sin(q1s(i));
    x_j2 = L1*cos(q1s(i))+L2*cos(q1s(i)+q2s(i));
    y_j2 = L1*sin(q1s(i))+L2*sin(q1s(i)+q2s(i));
    x_j2s(i) = x_j2;
    y_j2s(i) = y_j2;
    if (rem(time(i),0.5)==0)
        grid on
        grid minor
        xlabel ('x(m)')
        ylabel ('y(m)')
        title(['Robot config at t=',num2str(time(i)),'s'])
        plot([0, x_j1], [0, y_j1],'Linewidth',1,'Color',[0, 0.4470, 0.7410]);
        plot([x_j1, x_j2], [y_j1, y_j2],'Linewidth',1,'Color',[0.8500, 0.3250, 0.0980]);
        plot(x_j2, y_j2,'o','MarkerSize',2,'MarkerFaceColor',[0.8500, 0.3250, 0.0980]);
 
        axis equal
        axis([-0.4 1.6 -0.6 1.2])
%         s=hgexport('readstyle','yyy');
%         fnam=['t5_',num2str(k)]; % your file name
%         s.Format = 'eps'; %I needed this to make it work but maybe you wont.
%         hgexport(gcf,fnam,s);
%         k = k+1;
    end
end

figure
plot(time,dos,'kx');
grid on
grid minor
xlabel('time(s)')
ylabel('distance from obstacle (m)')
title('Obstacle distance from end-effector')
axis([0 5 0 0.4])

%% q1,q2 plot 
figure
plot(time,q1s*180/pi,'-o', time,q2s*180/pi,'-o')
grid on
grid minor
xlabel ('time (s)')
ylabel ('joint angle displacement (degree)')
title ('q_1,q_2 vs t')
legend('q_1','q_2')

%% q1d,q2d plot 
figure
plot(time,qdot1s*180/pi,'-o', time,qdot2s*180/pi,'-o')
grid on
grid minor
xlabel ('time (s)')
ylabel ('joint angle velocity (degree/s)')
title ('qd_1,qd_2 vs t')
legend('qd_1','qd_2')

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
plot(pDx(1),pDx(2),'x');
% detected points of obstacle
plot(xOs,yOs,'kx','MarkerSize',2);
% mapping of obstacle circle using fitting tools
angle = 0:pi/180:2*pi;
xOs_map = RObs*cos(angle)+xcObs;
yOs_map = RObs*sin(angle)+ycObs;
plot(xOs_map,yOs_map,'m-');

axis equal
grid on
grid minor
axis([-1.6 1.6 -1.6 1.6])
h = animatedline('MaximumNumPoints',2,'Linewidth',2,'Color',[0, 0.4470, 0.7410]);
g = animatedline('MaximumNumPoints',2,'Linewidth',2,'Color',[0.8500, 0.3250, 0.0980]);
[A,map] = rgb2ind(frame2im(getframe),512);
imwrite(A,map,'task5.gif','LoopCount',65535,'DelayTime',0.02);

for i=1:length(q1s)
    addpoints(h,0,0);
    addpoints(h,p1x(i),p1y(i));
    addpoints(g,p1x(i),p1y(i));
    addpoints(g,p2x(i),p2y(i));
    drawnow
    if(mod(i,25)==0)
        [A,map] = rgb2ind(frame2im(getframe),512);
        imwrite(A,map,'task5.gif','WriteMode','append','DelayTime',0.05);
    end
end