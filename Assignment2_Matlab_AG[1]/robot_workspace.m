function [x0,y0,z0] = robot_workspace(q1L,q2L,q3L,q4L,q5L,d1m,d2m,d3m,d4m,d5m,TE0_raw,res)
%% description
%  Try actually avoid using this function, use the 2-d version first
%  This is super slow
%  plots the 3d workspace of the specific 5-DOF robot designed 
%  for picking up the jenga tower.
%  q5 is also irrelvant as it only orientates the gripper -90 to 90 degrees

syms d1 d2 d3 d4 d5 d6 q1 q2 q3 q4 q5

%% input
%  &param q1L,q2L,q3L,q4L angle limits for each joints
%  &param d1,d2,d3,d4,d5  length
%  &param TE0, transformation matrix from end effector frame to 0
%  &param Res, resolution of workspace plot

% sub in dimensions first
TE0_raw = subs(TE0_raw,{d1,d2,d3,d4,d5,q5},{d1m,d2m,d3m,d4m,d5m,0});                   
%resolution of plot in degrees
q1span = linspace(q1L(1),q1L(2),res);
q2span = linspace(q2L(1),q2L(2),res);
q3span = linspace(q3L(1),q3L(2),res);
q4span = linspace(q4L(1),q4L(2),res);

% q1span = linspace(q1L(1),q1L(2),res);
% q2span = linspace(q2L(1),q2L(2),res);
% q3span = linspace(q3L(1),q3L(2),res);
% q4span = linspace(q4L(1),q4L(2),res);
% q1span = q1L(1):res:q1L(2);
% q2span = q2L(1):res:q2L(2);
% q3span = q3L(1):res:q3L(2);
% q4span = q4L(1):res:q4L(2);
% mesh_size = length(q1span)*length(q2span)*length(q3span)*length(q4span);

L1 = length(q1span);
L2 = length(q2span);
L3 = length(q3span);
L4 = length(q4span);

%initialize output
x0 = zeros(L1*L2*L3*L4,1);
y0 = zeros(L1*L2*L3*L4,1);
z0 = zeros(L1*L2*L3*L4,1);

for i=1:L1
    for j=1:L2
        for k=1:L3
            for l=1:L4
                 TE0 = subs(TE0_raw,{q1,q2,q3,q4},{q1span(i),q2span(j),q3span(i),q4span(k)});
                [x,y,z] = transform3d(0,0,0,TE0);
                x0(l+(k-1)*L4+(j-1)*L4*L3+(i-1)*L4*L3*L2) = x;
                y0(l+(k-1)*L4+(j-1)*L4*L3+(i-1)*L4*L3*L2) = y;
                z0(l+(k-1)*L4+(j-1)*L4*L3+(i-1)*L4*L3*L2) = z;
            end
        end
    end
end
                
                

end