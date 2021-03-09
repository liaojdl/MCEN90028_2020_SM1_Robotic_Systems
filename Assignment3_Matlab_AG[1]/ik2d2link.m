function [q_up,q_down] = ik2d2link(x,y,L1,L2)
%  author: Jiawei Liao 756560 liao2@student.unimelb.edu.au
%  version: Apr 2020
%% description
%  inverse kinematics of 2d 2link robot, takes in robot origin, 
%  end-effector position and computes joint angles
%% input
%  &param x,y [x,y] coordinate of robot relative to its origin
%  &param L1, length of link1
%  &param L2, length of link2
%% output
%  &param q1,q2, angle displacement in radians of joint 1 and joint 2

% q2 via cosine rule
q2a = acos((x^2+y^2-(L1^2+L2^2))/(2*L1*L2));
q2b = -q2a;

% delta
delta1 = abs(pi-q2a);
delta2 = abs(pi-q2b);

% alpha
alpha1 = asin(L2*sin(delta1)/sqrt(x^2+y^2));
alpha2 = asin(L2*sin(delta2)/sqrt(x^2+y^2));

% beta
beta = atan(y/x);
if (x<0 && y>0)
    beta = atan(y/x)+pi;
elseif (x<0 && y<0)
    beta = atan(y/x)-pi;
end
% q1
q1a = beta-alpha1;
q1b = beta-alpha2;   

% elbow up
if (q2a>0)
    q_up = [q1a,q2a];
    q_down = [q1b,q2b];
else
% elbow down
    q_up = [q1b,q2b];
    q_down = [q1a,q2a];
end