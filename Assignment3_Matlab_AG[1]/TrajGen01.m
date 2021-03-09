function SegmentCoeff = TrajGen01(init,final,tfinal)
%  author: Jiawei Liao 756560 liao2@student.unimelb.edu.au
%  version: Apr 2020
%% description
%  A trajectory generation function for Task1 of Assignment 3, MCEN90028
%  It takes in the initial and final conditions of the segments plus the
%  time duration for each segment and computes the  coefficients
%  of the polynomial trajectory

%% input
%  &param init, is the ((P+1)/2) X kmax matrix containing the initial 
%               conditions for all the segments, for example, for cubic 
%               polynomials (P = 3), init will contain the initial 
%               position, initial velocity (therefore, 
%               ((P + 1)/2) = ((3 + 1)/2) = 2 for all kmax segments),
%               therefore init will be a 2 by kmax matrix.
%  &param final, final is the ((P + 1)/2)  kmax matrix containing the 
%               final conditions for all the segments.
%  &param tfinal, is an array of size kmax X 1, containing the time
%  duration of each segment of the trajectory.
%% output
%  &param SegmentCoeff, (P+1) X K_max matrix containing the coefficients
%               of the polynomials for all segments of the trajectory

%% Check dimension consistency in inputs
[Linit,Winit] = size(init);
[Lfinal,Wfinal] = size(final);
Ltfinal = length(tfinal);
if ((Linit~=Lfinal)||(Winit~=Wfinal)||(Winit~=Ltfinal))
    fprintf("Inconsistent Input Dimensions!");
    return
end

%% load essential parameters
% number of segments
kMax = Winit;
% order of polynomial
P = Linit*2-1;
% number of polynomial coefficients
M = P+1;

%% Getting things setup for each segment
% form A as a symbolic square matrix 
syms ft
A = sym(eye(M));
poly = sym(zeros(1,M));
for i=1:M
   poly(i) = ft^(i-1);  
end
% derivatives
polydiff = poly;
A(M/2+1,:) = poly;
for i=2:(M/2)
    polydiff = diff(polydiff);
    polydiff0 = subs(polydiff,{ft},{0});
    A(i,:) = polydiff0;
    A(M/2+i,:) = polydiff;
end
%% output initialisation
SegmentCoeff = zeros(M,kMax);

%% iterating to find coefficients for each segment
% coeff for each segment = Ax\xx
for i=1:kMax
    Ax = subs(A,{ft},{tfinal(i)});
    Xx = [init(:,i);final(:,i)];
    SegmentCoeff(:,i) = Ax\Xx;
end

end

