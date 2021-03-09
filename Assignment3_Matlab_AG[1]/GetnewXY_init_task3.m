function [Xi,Yi,Xf,Yf,tSegx] = GetnewXY_init_task3(Lbd,Ldc,pA,pB,pC,pD,tSeg,mode)
%  author: Jiawei Liao 756560 liao2@student.unimelb.edu.au
%  version: Apr 2020
%% description
%  Get a new initial condition based on mode and point of D, task 2
%  specific
%% input
%  &param pA,pB,pC,pD, [x,y] of points
%  &param Lbd,Ldc, eucledean distance from B to D, and D to C
%  &param tseg, the time segments array
%  &param mode, 'straight','smooth', straight uses 0 velocity constraint,
%  'smooth' uses interpolation

%% output
%  &param initial conditions and constraints, xi,yi,xf,yf, 

%a simple smart way to automatically set intermediate velocity constraint
        %and also intermediate segment time composition.
        %step 1: compute segment trajectory length
        %step 2: the porportional time given to the segment is proportional to 
                %the proportional length to the entire trajectory 
        %step 1: for each segment, compute average velocity for x and y
        %step 2: intermediate point velocity constraint set as the average of 
        
        
% time of segment BD,and DC based on BD,DC length
tBD = tSeg(2)*Lbd/(Lbd+Ldc);
tBD = round(tBD,2);
tDC = tSeg(2)-tBD;
tSegx = [3;tBD;tDC];

%avg x,y velocity from A to B
vxAB = (pB(1)-pA(1))/tBD;
vyAB = (pB(2)-pA(2))/tBD;
%avg x,y velocity from B to D
vxBD = (pD(1)-pB(1))/tBD;
vyBD = (pD(2)-pB(2))/tBD;
%avg x,y velocity from D to C
vxDC = (pC(1)-pD(1))/tBD;
vyDC = (pC(2)-pD(2))/tBD;

switch mode
    case 'smooth'
        
        %the mean of the two segment average velocity, respective for x,y.
        %avg x,y velocity from D to C
        %velocity at D is set as mean velocity from B to D and D to C
        Xi = [pA(1),pB(1),pD(1);0,(vxAB+vxBD)/2,(vxBD+vxDC)/2];
        Xf = [pB(1),pD(1),pC(1);(vxAB+vxBD)/2,(vxBD+vxDC)/2,0];
        Yi = [pA(2),pB(2),pD(2);0,(vyAB+vyBD)/2,(vyBD+vyDC)/2];
        Yf = [pB(2),pD(2),pC(2);(vyAB+vyBD)/2,(vyBD+vyDC)/2,0];
    case 'straight'
        Xi = [pA(1),pB(1),pD(1);0,0,0];
        Xf = [pB(1),pD(1),pC(1);0,0,0];
        Yi = [pA(2),pB(2),pD(2);0,0,0];
        Yf = [pB(2),pD(2),pC(2);0,0,0];
end

end