%% MCEN9028 Robotics System Assignment 3 task 2
%%
% 
%  main file to operate and test assignment 3 task 2 for MCEN90028 Robotics Systems
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
offset = ro/20;
% plot resolution, this should be fixed
shift_step = ro/50;
%obstacle avoidance shifting maximum iteration
maxiter = 50;
% plot resolution
step = 0.01;

%% Task 2, object avoidance between B and C via viapoint D

%determine the viapoint
[pD1,pD2] = findviapoint(pB,pC,pOBS,ro);

%check viapoint is needed
if (isnan(pD1(1)) && isnan(pD1(1)))
    fprintf("No need for point D!");
    return
end

% distance squared traveled via viapoint 1
Lbd_a = sqrt((pD1(1)-pB(1))^2+(pD1(2)-pB(2))^2);
Ldc_a = sqrt((pD1(1)-pC(1))^2+(pD1(2)-pC(2))^2);
d_a = Lbd_a+Ldc_a;
% distance squared traveled via viapoint 2
Lbd_b = sqrt((pD2(1)-pB(1))^2+(pD2(2)-pB(2))^2);
Ldc_b = sqrt((pD2(1)-pC(1))^2+(pD2(2)-pC(2))^2);
d_b = Lbd_b+Ldc_b;

% chooses the one with less distance travelled
if (d_a<d_b)
    pD = pD1;
else
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
    Lbd = sqrt((pD(1)-pB(1))^2+(pD(2)-pB(2))^2);
    Ldc = sqrt((pD(1)-pC(1))^2+(pD(2)-pC(2))^2);
    [Xi,Yi,Xf,Yf,tSegx] = GetnewXY_init_task2(Lbd,Ldc,pA,pB,pC,pD,tSeg,'straight');
    % % Get the polynomial co-efficients for x(t) and y(t)
    xtCoeff = TrajGen01(Xi,Xf,tSegx);
    ytCoeff = TrajGen01(Yi,Yf,tSegx);
    % get x(t), y(t), xv(t), yv(t)
    [xt,yt] = Getxt_yt(tSegx,xtCoeff,ytCoeff,step);
    [xv,yv] = Getxv_yv(tSegx,xtCoeff,ytCoeff,step);
    
    flag = 0;
    for i=1:length(xt)
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


xtCoeff
ytCoeff
tSegx
Xi
Xf
Yi
Yf
pD
%% plotting
% plot axes limits
axlim1 = [0 8 0 18];
axlim2 = [0 8 -4 4];
axlim3 = [0 15 0 10];
% the three plots
[xt,yt] = Getxt_yt(tSegx,xtCoeff,ytCoeff,step);
[xv,yv] = Getxv_yv(tSegx,xtCoeff,ytCoeff,step);
plotxyt_xvyvt_xvyv(Xi,Xf,Yi,Yf,xt,yt,xv,yv,tSegx,step,axlim1,axlim2,axlim3);
% add circle
theta = 0:pi/100:2*pi;
cx = ro*cos(theta)+pOBS(1);
cy = ro*sin(theta)+pOBS(2);
plot(cx,cy);