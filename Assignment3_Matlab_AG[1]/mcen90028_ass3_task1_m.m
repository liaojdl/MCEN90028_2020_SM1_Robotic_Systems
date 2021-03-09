%% MCEN9028 Robotics System Assignment 3 Task 1
%%
% 
%  main file to operate and test assignment 3 task 1 for MCEN90028 Robotics Systems
%
%% Version 1.0 2020 Apr, Jiawei Liao, 756560, <mailto:liao2@student.unimelb.edu.au <mailto:liao2@student.unimelb.edu.au liao2@student.unimelb.edu.au>>
%% Full clean up

clc; clear all; close all;
%% General problem definitions and variables
% point of A,B,C and center of obstacle

pA = [2,5];
pB = [5,8];
pC = [14,7];
pOBS = [10,7];
% time of each segment
tSeg = [3;5];
% plot resolution, this should be fixed
step = 0.01;
%% Task 1: trajectory generation algorithm
%% 1a problem formulation.

% initial and final conditions
Xi = [pA(1),pB(1);0,0.25];
Xf = [pB(1),pC(1);0.25,0];
Yi = [pA(2),pB(2);0,0.25];
Yf = [pB(2),pC(2);0.25,0];

% Get the polynomial co-efficients for x(t) and y(t)
xtCoeff = TrajGen01(Xi,Xf,tSeg)
ytCoeff = TrajGen01(Yi,Yf,tSeg)

%% 1b trajectory and velocity obtained and plotted
% plot axes limits
axlim1 = [0 8 0 18];
axlim2 = [0 8 -4 4];
axlim3 = [0 15 0 10];
[xt,yt] = Getxt_yt(tSeg,xtCoeff,ytCoeff,step);
[xvt,yvt] = Getxv_yv(tSeg,xtCoeff,ytCoeff,step);
plotxyt_xvyvt_xvyv(Xi,Xf,Yi,Yf,xt,yt,xvt,yvt,tSeg,step,axlim1,axlim2,axlim3);