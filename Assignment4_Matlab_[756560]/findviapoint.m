function [pD1,pD2] = findviapoint(pB,pC,pOBS,ro)
%  author: Jiawei Liao 756560 liao2@student.unimelb.edu.au
%  version: Apr 2020
%% description
%  find a viapoint between p1 and p2 where there is a circular obstacle 
%  centered at po of radius ro such that the path from p1 to viapoint and 
%  then to p2 is of optimal path distance.
%  This is achieved by
%  1. if a straight line segment betwen p1 and p2 does not cross the
%  obstacle, return viapoint as [Nan,Nan]
%  2. otherwise compute the two tangent segments of p1 to obstacle, two
%     tangent segments of p2 to obstacle. Each sets of tangent lines
%     will meet as one point to give the viapoint. 
%  3. output the viapoint (one above and one below) that has the minimum
%     distance
%% input
%  &param pB,pC, [x,y] coordinate of two points
%  &param po, [x,y] coordinate of the centre of the circular obstacle, the 
%             obstacle should ideally be between p1 and p2's x coordinate
%  &param ro, radius of the circular obstacle
%  &param pref, preference of viapoint, either 'top', 'bot' or 'both'
%% output
%  &param pD, [x,y] coordiante of the viapoint, [NaN,NaN] if not
%               needed
%  &param L1o,L2o, the eucledean distance from P1 to viapoint, and
%  the eucledean distance from viapoint to P2

% initialise output as NaN
pD1 = [NaN,NaN];
pD2 = [NaN,NaN];

%% check that point b and c are not in obstacle
[~,pB_clear] = isincircle(pB,pOBS,ro);
if (pB_clear ~= 0)
    fprintf("point B is inside the obstacle circle!");
    return
end
[~,pC_clear] = isincircle(pC,pOBS,ro);
if (pC_clear ~= 0)
    fprintf("point C is inside the obstacle circle!");
    return
end

%% check if obstacle is really in the way

% construct the straight line segment BC, resolution of ro/50
stepsize = ro/100;
Nsteps = round(abs(pC(2)-pB(2))+abs(pC(1)-pB(1))/(stepsize));
x_steps = linspace(pB(1),pC(1),Nsteps);
y_steps = linspace(pB(2),pC(2),Nsteps);

% flag for intersection with the obstacle
flag = 0;
for i=1:length(x_steps)
    if ((x_steps(i)-pOBS(1))<0.05)
        linex = x_steps(i);
        liney = y_steps(i);
    end
    [~,BC_clear] = isincircle([x_steps(i),y_steps(i)],pOBS,ro);
    if (BC_clear~=0)
        flag = 1;
    end
end

% the BC does not cross circle at all, no need for via point
if (flag==0)
    if (liney<(pOBS(2)-ro))
        fprintf("No need for via point betwen B and C!")
        return
    end
end

%% if viapoint is needed, get the two possible candidates
syms x y
% equation of obstacle circle
cof = (x-pOBS(1))^2+(y-pOBS(2))^2==ro^2;
% distance between p1 and po
d1o = (pB(1)-pOBS(1))^2+(pB(2)-pOBS(2))^2;
% distance between p2 and po
d2o = (pC(1)-pOBS(1))^2+(pC(2)-pOBS(2))^2;
% equation of circle of radius LBo at point B
eqn1 = (x-pB(1))^2+(y-pB(2))^2==d1o;
% equation of circle of radius LCo at point C
eqn2 = (x-pC(1))^2+(y-pC(2))^2==d2o;

% the tangental points on the obstacle, from p1,p2 respectively
[x1,y1] = solve(eqn1,cof,x,y);
[x2,y2] = solve(eqn2,cof,x,y);
x1 = double(x1);y1 = double(y1);x2 = double(x2);y2 = double(y2);

% get equations of the two pairs of two tangental lines
t1a = poly2sym(polyfit([pB(1),x1(1)],[pB(2),y1(1)],1));
t1b = poly2sym(polyfit([pB(1),x1(2)],[pB(2),y1(2)],1));
t2a = poly2sym(polyfit([pC(1),x2(1)],[pC(2),y2(1)],1));
t2b = poly2sym(polyfit([pC(1),x2(2)],[pC(2),y2(2)],1));

% get intersection between a pair of tangental lines, up to 4
[xd1,yd1] = solve(y==t1a,y==t2a);
[xd2,yd2] = solve(y==t1b,y==t2b);
[xd3,yd3] = solve(y==t1a,y==t2b);
[xd4,yd4] = solve(y==t1b,y==t2a);

xD = [xd1,xd2,xd3,xd4];
yD = [yd1,yd2,yd3,yd4];

% only the two point with the least distance to centre of obstacle
% are relevant
[d1,~] = isincircle([xd1,yd1],pOBS,ro);
[d2,~] = isincircle([xd2,yd2],pOBS,ro);
[d3,~] = isincircle([xd3,yd3],pOBS,ro);
[d4,~] = isincircle([xd4,yd4],pOBS,ro);

% sort the distance
dD = [d1,d2,d3,d4];
[~,idD] = sort(dD);

%% the two relevant points are then found as the two nearest
xda = xD(idD(1)); xda = double(xda);
xdb = xD(idD(2)); xdb = double(xdb);
yda = yD(idD(1)); yda = double(yda);
ydb = yD(idD(2)); ydb = double(ydb);

%% output the two potential candidates
pD1 = [xda,yda];
pD2 = [xdb,ydb];
end