%% MCEN9028 Robotics System Assignment 1
%  main file to operate and test parameters of a 5 DOF robotic
%  arm using forward and inverse kinematics.

%% Version 1.0 2020 Mar, Jiawei Liao, 756560, liao2@student.unimelb.edu.au

%% Full clean up
clc; clear all; close all;

%% Define variables for robot parameters
% let ls represent joint length, let as represent joint angles
syms d1 d2 d3 d4 d5 d6 q1 q2 q3 q4 q5
    
%% DH table and homogeneous transitional matrices:
%DH Table
Mdh = [0 0 d1 (q1+pi/2);
        0 pi/2 0 q2;
        d2 0 0 q3;
        d3 0 0 (q4+pi/2);
        0 pi/2 d4 q5;
        0 pi -d5 0];   
%transformation matrices between frames, 6 in total, 24*4 matrix structure
Tdh = T_linksrad(Mdh)
%some simplification needed
Tdh = subs(Tdh,{cos(q1 + pi/2),sin(q1 + pi/2),...
    cos(q4 + pi/2),sin(q4 + pi/2)},...
    {-sin(q1),cos(q1),...
    -sin(q4),cos(q4)})
T10 = Tdh(1:4,:);
T21 = Tdh(5:8,:);
T32 = Tdh(9:12,:);
T43 = Tdh(13:16,:);
% T54 = Tdh(17:20,:);
% TE5 = Tdh(21:24,:);
%transfomration matrices from all non-inertial frames to frame 0
T4_rs0 = T_rs0(Tdh(1:16,:));
T20 = T4_rs0(5:8,:)
T30 = T4_rs0(9:12,:)
T40 = T4_rs0(13:16,:)


R_OW = T40*[0;0;0;1]
R_OW = R_OW(1:3)
JVW = [diff(R_OW,q1),diff(R_OW,q2),diff(R_OW,q3)]