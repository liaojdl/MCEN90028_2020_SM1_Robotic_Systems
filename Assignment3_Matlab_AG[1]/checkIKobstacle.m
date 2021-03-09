function validity = checkIKobstacle(pB,pC,pO,ro,L1,L2,origin)
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
[qCup,qCdown]=ik2d2link(pC(1)-origin(1),pC(2)-origin(2),L1,L2);

%q1 rotation for pointB and point C, two pairs
q1_Bup = qBup(1);
% q2_Bup = qBup(2);
q1_Bdown = qBdown(1);
% q2_Bdown = qBdown(2);
q1_Cup = qCup(1);
% q2_Cup = qCup(2);
q1_Cdown = qCdown(1);
% q2_Cdown = qCdown(2);

%postition of joint 2 for the above 4 ik solutions
[J2x_Bup,J2y_Bup] = forward2d2link(q1_Bup,L1,origin);
[J2x_Bdown,J2y_Bdown] = forward2d2link(q1_Bdown,L1,origin);
[J2x_Cup,J2y_Cup] = forward2d2link(q1_Cup,L1,origin);
[J2x_Cdown,J2y_Cdown] = forward2d2link(q1_Cdown,L1,origin);

%% check if ik solution is obstructed by obstacle, resolution of ro/100
stepsize = ro/20;
NL1 = round(L1/stepsize);
NL2 = round(L2/stepsize);
%construct the linkage segments for each pair of solutions
L1_Bup = [linspace(origin(1),J2x_Bup,NL1);linspace(origin(2),J2y_Bup,NL1)];
L1_Bdown = [linspace(origin(1),J2x_Bdown,NL1);linspace(origin(2),J2y_Bdown,NL1)];
L1_Cup = [linspace(origin(1),J2x_Cup,NL1);linspace(origin(2),J2y_Cup,NL1)];
L1_Cdown = [linspace(origin(1),J2x_Cdown,NL1);linspace(origin(2),J2y_Cdown,NL1)];

L2_Bup = [linspace(J2x_Bup,pB(1),NL1);linspace(J2y_Bup,pB(2),NL2)];
L2_Bdown = [linspace(J2x_Bdown,pB(1),NL1);linspace(J2y_Bdown,pB(2),NL2)];
L2_Cup = [linspace(J2x_Cup,pC(1),NL1);linspace(J2y_Cup,pC(2),NL2)];
L2_Cdown = [linspace(J2x_Cdown,pC(1),NL1);linspace(J2y_Cdown,pC(2),NL2)];

%assume no obstuction
Conup = 0;
Condown = 0;

% check link 1
for i=1:NL1
    [~,state_L1Bup] = isincircle([L1_Bup(1,i),L1_Bup(2,i)],pO,ro);
    if (state_L1Bup~=0)
        Conup = 1;
    end
    [~,state_L1Bdown] = isincircle([L1_Bdown(1,i),L1_Bdown(2,i)],pO,ro);
    if (state_L1Bdown~=0)
        Condown = 1;
    end    
    [~,state_L1Cup] = isincircle([L1_Cup(1,i),L1_Cup(2,i)],pO,ro);
    if (state_L1Cup~=0)
        Conup = 1;
    end
    [~,state_L1Cdown] = isincircle([L1_Cdown(1,i),L1_Cdown(2,i)],pO,ro);
    if (state_L1Cdown~=0)
        Condown = 1;
    end    
end

% check link 2
for i=1:NL2
    [~,state_L2Bup] = isincircle([L2_Bup(1,i),L2_Bup(2,i)],pO,ro);
    if (state_L2Bup~=0)
        Conup = 1;
    end
    [~,state_L2Bdown] = isincircle([L2_Bdown(1,i),L2_Bdown(2,i)],pO,ro);
    if (state_L2Bdown~=0)
        Condown = 1;
    end    
    [~,state_L2Cup] = isincircle([L2_Cup(1,i),L2_Cup(2,i)],pO,ro);
    if (state_L2Cup~=0)
        Conup = 1;
    end
    [~,state_L2Cdown] = isincircle([L2_Cdown(1,i),L2_Cdown(2,i)],pO,ro);
    if (state_L2Cdown~=0)
        Condown = 1;
    end    
end

%% outputting
validity = 'none';
if (Conup == 0 && Condown == 0)
    validity = 'both';
elseif (Conup == 1 && Condown == 0)
    validity = 'down';
elseif (Conup == 0 && Condown == 1)
    validity = 'up';
end