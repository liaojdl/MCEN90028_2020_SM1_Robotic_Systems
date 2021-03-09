%% MCEN9028 Robotics System Assignment 3 task 3
%%
% 
%  main file to operate and test assignment 3 task 3 for MCEN90028 Robotics Systems
%
%% Version 1.0 2020 Apr, Jiawei Liao, 756560, <mailto:liao2@student.unimelb.edu.au liao2@student.unimelb.edu.au>
%% Full clean up

clc; clear all; close all;

%% General problem definitions and variables
% point of A,B,C and center of obstacle
pA = [2,5];
pB = [5,8];
pC = [14,7];
pOBS = [10,7];

%% some other to play with
% pA = [-2,-5];
% pB = [-5,-8];
% pC = [-14,-7];
% pOBS = [-10,-7];
% pA = [6,-2];
% pB = [6,0];
% pC = [6,8];
% pOBS = [6,3];


% time of each segment
tSeg = [3;5];
%obstacle radius
ro = 1.5;
%obstacle avoidance offset shift step size
offset = 0;
% plot resolution, this should be fixed
shift_step = ro/50;
%obstacle avoidance shifting maximum iteration
maxiter = 50;
% plot resolution
step = 0.01;
% robot origin
origin = [0,0];
% link lengths
L1 = 9;
L2 = 9;

%% Task 3: obstacle avoidance considering robot kinematics
%% 2-Link robot workspace

% %determine the 2 possible viapoint
[pD1,pD2] = findviapoint(pB,pC,pOBS,ro);
%check viapoint is needed
if (isnan(pD1(1)) && isnan(pD1(1)))
    fprintf("No need for point D!");
    return
end

% check viable config at B and C
validity = checkIKobstacle(pB,pC,pOBS,ro,L1,L2,origin)
if (strcmp(validity,'none')) 
    fprintf("there is no way to find a via point to work!");
    return
end
% determie the possible config at two viapoints
[vD1,vD2] = getOptimalpointD(pD1,pD2,pOBS,ro,L1,L2,origin)

% get the correct viapoint
if (strcmp(vD1,validity) || strcmp(vD1,'both'))
    pD = pD1;
end
if (strcmp(vD2,validity) || strcmp(vD2,'both'))
    pD = pD2;
end

%slope of point pOBS to point D and relative position of D to pOBS
rise_OBSD = (pD(2)-pOBS(2));
run_OBSD = (pD(1)-pOBS(1));
slope_OBSD = rise_OBSD/run_OBSD;


%% Getting the coeffiencts and set constraints
% Get new some constraints
% straight mode sets all velocity constraint to 0 where possible
% smooth mode uses interpolation to get smoothed trajectory for less abrupt
% velocity changes

syms x y
iter = 0;
goal = 0;
while ((goal~=1) &&(iter<maxiter))
    iter = iter+1
    Lbd = sqrt((pD(1)-pB(1))^2+(pD(1)-pB(2))^2);
    Ldc = sqrt((pD(1)-pC(1))^2+(pD(2)-pC(2))^2);
    [Xi,Yi,Xf,Yf,tSegx] = GetnewXY_init_task3(Lbd,Ldc,pA,pB,pC,pD,tSeg,'smooth');
    % % Get the polynomial co-efficients for x(t) and y(t)
    xtCoeff = TrajGen01(Xi,Xf,tSegx);
    ytCoeff = TrajGen01(Yi,Yf,tSegx);
    % get x(t), y(t), xv(t), yv(t)
    [xt,yt] = Getxt_yt(tSegx,xtCoeff,ytCoeff,step);
    [xv,yv] = Getxv_yv(tSegx,xtCoeff,ytCoeff,step);
      
    flag = 0;
    for i=1:length(xt)
        % ik solution of robot
        [qBup,qBdown]=ik2d2link(xt(i)-origin(1),yt(i)-origin(2),L1,L2);
        switch validity
            case 'both'
                q1 = qBup(1); 
                q2 = qBup(2); 
            case 'up'
                q1 = qBup(1); 
                q2 = qBup(2); 
            case 'down'
                q1 = qBdown(1); 
                q2 = qBdown(2);
        end
        % check robot links
        robot_clear = checkIKobstacle1point([xt(i),yt(i)],pOBS,ro,L1,L2,origin,validity);
        if (robot_clear ~= 0)
            flag = 1;
            break;
        end 
        %check end effector
        [~,state] = isincircle([xt(i),yt(i)],pOBS,ro);
        if (state ~= 0)
            flag = 1;
            break;
        end
    end
    if (flag == 0)
        goal = 1;
        break;
    else
        if (run_OBSD>0)
            x_shift = solve(x^2+(slope_OBSD*x)^2==shift_step^2,x>0,x);
        elseif (run_OBSD<0)
            x_shift = solve(x^2+(slope_OBSD*x)^2==shift_step^2,x<0,x);
        end
        x_shift = double(x_shift);
        pD = [pD(1)+x_shift,pD(2)+x_shift*slope_OBSD];
    end
end

% plot axes limits
axlim1 = [0 8 0 18];
axlim2 = [0 8 -4 4];
axlim3 = [0 15 0 10];
% the three plots
% [xt,yt] = Getxt_yt(tSegx,xtCoeff,ytCoeff,step);
% [xv,yv] = Getxv_yv(tSegx,xtCoeff,ytCoeff,step);
plotxyt_xvyvt_xvyv(Xi,Xf,Yi,Yf,xt,yt,xv,yv,tSegx,step,axlim1,axlim2,axlim3);
% add circle
theta = 0:pi/100:2*pi;
cx = ro*cos(theta)+pOBS(1);
cy = ro*sin(theta)+pOBS(2);
plot(cx,cy);

xtCoeff
ytCoeff
tSegx
Xi
Xf
Yi
Yf
pD

%% animating the robot
%robot angle displacment over time
q1t = zeros(1,length(xt));
q2t = zeros(1,length(xt));

for i=1:length(xt)
    [q_up,q_down]=ik2d2link(xt(i)-origin(1),yt(i)-origin(2),L1,L2);
%     q1t(i) = q1(2);
%     q2t(i) = q2(2);
    if (strcmp(validity,'up'))  
        q1t(i) = q_up(1);
        q2t(i) = q_up(2);
    elseif (strcmp(validity,'down'))
        q1t(i) = q_down(1);
        q2t(i) = q_down(2);
    end
end

p2x = L1*cos(q1t)+origin(1);
p2y = L1*sin(q1t)+origin(2);

h = animatedline('MaximumNumPoints',3);
[A,map] = rgb2ind(frame2im(getframe),256);
imwrite(A,map,'3.gif','LoopCount',65535,'DelayTime',0.01);

for i=1:length(xt)
    addpoints(h,origin(1),origin(2));
    addpoints(h,p2x(i),p2y(i));
    addpoints(h,xt(i),yt(i));
    drawnow
    if(mod(i,5)==0)
        [A,map] = rgb2ind(frame2im(getframe),512);
        imwrite(A,map,'3.gif','WriteMode','append','DelayTime',0.05);
    end
end
clearpoints(h);