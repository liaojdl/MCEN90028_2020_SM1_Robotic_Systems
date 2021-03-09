function [distance,state] = isincircle(p1,po,ro)
%  author: Jiawei Liao 756560 liao2@student.unimelb.edu.au
%  version: Apr 2020
%% description
%  check if a point is in, out of a circle or on circumference
%% input
%  &param p1, [x,y] of point 1
%  &param po, [x,y] coordinate of the centre of the circular obstacle
%  &param ro, radius of the circular obstacle
%% output
%  &param state, 0,1,2, 0 out of circle, 1 inside circle, 2 on cicumference
%  &param distance, distance to centre of circle

distance = (p1(1)-po(1))^2+(p1(2)-po(2))^2;
if (distance == ro^2)
    state = 2;
elseif (distance < ro^2)
    state = 1;
else
    state = 0;
end
    
end