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
Mdh = [0 0 d1 (q1+90);
        0 90 0 q2;
        d2 0 0 q3;
        d3 0 0 (q4+90);
        0 90 d4 q5;
        0 180 -d5 0];   
%transformation matrices between frames, 6 in total, 24*4 matrix structure
Tdh = T_links(Mdh);
%transfomration matrices from all non-inertial frames to frame 0
Trs0 = T_rs0(Tdh);
%transformation matrix of {E} to {0}
TE0_raw = Trs0(end-3:end,1:4);

%% subsititute of actual parameter values to validate TE0 Forward dynamics
% zero configuration
TE0_config0 = subs(TE0_raw,{q1,q2,q3,q4,q5},{0,0,0,0,0})
% configuration 1
TE0_config1 = subs(TE0_raw,{q1,q2,q3,q4,q5},{30,0,0,-90,0})
% configuration 2
TE0_config2 = subs(TE0_raw,{q1,q2,q3,q4,q5},{0,45,-90,-45,90})

%% deciding dimensions
d1m = 0.15;
d2m = 0.2;
d3m = 0.25;
d4m = 0.1;
d5m = 0.05;



%% plot robot workspace in two views, top and side.
robot_simplews_plot([-90,90],[0,120],[-150,-15],[-150,-15],[-90,90],d1m,d2m,d3m,d4m,d5m,20);

%% animate robot to designated joint angles, dont necessarily need to use
% robot_3dplot(45,30,-30,-30,0,d1m,d2m,d3m,d4m,d5m,Trs0);

%% 3-D workspace, use only once for validation, very very slow!!! 
% [xo,yo,zo] = robot_workspace([-90,90],[0,120],[-150,-15],[-150,-15],[-90,90],d1m,d2m,d3m,d4m,d5m,TE0_raw,8);
% scatter3(xo,yo,zo,'MarkerEdgeColor','k','MarkerFaceColor',[0 .75 .75]);
% plot3(1,1,1,'ro','LineWidth',25);

%% Inverse Kinematics validation
%zero config, different from last section, q4 is -90
% TE0_config0 = subs(TE0_raw,{q1,q2,q3,q4,q5,d1,d2,d3,d4,d5},{0,0,0,-90,0,d1m,d2m,d3m,d4m,d5m});
% [x0,y0,z0] = transform3d(0,0,0,TE0_config0)
% [~,Q20] = IK_robot([x0,y0,z0],d1m,d2m,d3m,d4m,d5m)
% %config 1
% TE0_config1 = subs(TE0_raw,{q1,q2,q3,q4,q5,d1,d2,d3,d4,d5},{30,0,0,-90,0,d1m,d2m,d3m,d4m,d5m});
% [x1,y1,z1] = transform3d(0,0,0,TE0_config1)
% [~,Q21] = IK_robot([x1,y1,z1],d1m,d2m,d3m,d4m,d5m)
% %config 2
% TE0_config2 = subs(TE0_raw,{q1,q2,q3,q4,q5,d1,d2,d3,d4,d5},{0,45,-90,-45,90,d1m,d2m,d3m,d4m,d5m});
% [x2,y2,z2] = transform3d(0,0,0,TE0_config2)
% [~,Q22] = IK_robot([x2,y2,z2],d1m,d2m,d3m,d4m,d5m)
% [Q1,Q2] = IK_robot([0.2,0.1,0.13],d1m,d2m,d3m,d4m,d5m);


%% Test vertices of the 8 vertices of the jenga tower
% assume jenga tower's vertices
tower_cor = [0,0,0;0,0.075,0;0.075,0,0;0.075,0.075,0;
             0,0,0.27;0,0.075,0.27;0.075,0,0.27;0.075,0.075,0.27];
tower_offset = [0,0.25,0];
tower_cor = tower_cor+tower_offset;

% produce solutions and plot in 3D
for i = 1:length(tower_cor)
    i
    [Q1,Q2] = IK_robot(tower_cor(i,:),d1m,d2m,d3m,d4m,d5m)
%   robot_3dplot(Q1(1),Q1(2),Q1(3),Q1(4),Q1(5),d1m,d2m,d3m,d4m,d5m,Trs0);
%   robot_3dplot(Q2(1),Q2(2),Q2(3),Q2(4),Q2(5),d1m,d2m,d3m,d4m,d5m,Trs0);
end