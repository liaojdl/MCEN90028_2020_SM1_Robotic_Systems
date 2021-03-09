function validity = checkIKobstacle1point(pB,pO,ro,L1,L2,origin,config)
%  author: Jiawei Liao 756560 liao2@student.unimelb.edu.au
%  version: Apr 2020

%% description
%  gets the ik solution of robot at point B and C, where a circular
%  obstacle or radius ro is centered at pO. checks if valid pairs of 
%  ik solution exists in the same manifold for both B and C and not in
%  contact with the obstacle.

%% input
%  &param pB,pC, [x,y] coordinate of two points B and C
%  &param pO, [x,y] coordinate of the centre of the circular obstacle, the 
%             obstacle should be strictly between p1 and p2's x coordinate
%  &param ro, radius of the circular obstacle
%  &param L1,L2 linkage lengths of robot
%  &param origin [x,y] coordinate of robot origin
%% output
%  &param validity, whether a viapoint can be obtained, 
%  'both' can be both on top or below circle, 'up' indicates elbow up op
%  'down' indicates elbow down op, 'none' indicates no way, obstacle is too 
%  big of a trouble.

%% get ik solution at point B and C
%% get ik solution at point B and C
[qBup,qBdown]=ik2d2link(pB(1)-origin(1),pB(2)-origin(2),L1,L2);

%q1 rotation for pointB and point C, two pairs
switch config 
    case 'up'
        q1_B = qBup(1);
    case 'down'
        q1_B = qBdown(1);
    case 'both'
        q1_B = qBup(1);
end

%postition of joint 2 for the above 4 ik solutions
[J2x,J2y] = forward2d2link(q1_B,L1,origin);

%% check if ik solution is obstructed by obstacle, resolution of ro/100
stepsize = ro/50;
NL1 = round(L1/stepsize);
NL2 = round(L2/stepsize);
%construct the linkage segments for each pair of solutions
L1_B = [linspace(origin(1),J2x,NL1);linspace(origin(2),J2y,NL1)];
L2_B = [linspace(J2x,pB(1),NL1);linspace(J2y,pB(2),NL2)];
%assume no obstuction
Conup = 0;
Condown = 0;

% check link 1
for i=1:NL1
    [~,state_L1Bup] = isincircle([L1_B(1,i),L1_B(2,i)],pO,ro);
    if (state_L1Bup~=0)
        Conup = 1;
    end 
end

% check link 2
for i=1:NL2
    [~,state_L2Bup] = isincircle([L2_B(1,i),L2_B(2,i)],pO,ro);
    if (state_L2Bup~=0)
        Conup = 1;
    end  
end

%% outputting
validity = 0;
if (Conup == 1 || Condown == 1)
    validity = 1;
end

end