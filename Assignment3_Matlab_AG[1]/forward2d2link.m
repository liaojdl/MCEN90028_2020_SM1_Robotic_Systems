function [rx,ry] = forward2d2link(q1,L1,origin)
%  author: Jiawei Liao 756560 liao2@student.unimelb.edu.au
%  version: Apr 2020
%% description
%  forward kinematics of 2d 2link robot, takes in robot origin, 
%  end-effector position and computes joint angles
%% input
%  &param q1, angle displacement in radians of joint 1 
%  &param L1, length of link1
%  &param origin, [x,y] of robot origin
%% output
%  &param x,y [x,y] coordinate of joint 2 of robot 

rx = L1*cos(q1)+origin(1);
ry = L1*sin(q1)+origin(2);

end