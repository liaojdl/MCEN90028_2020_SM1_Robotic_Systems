function [xt,yt] = Getxt_yt(tSeg,xtCoeff,ytCoeff,step)
%  author: Jiawei Liao 756560 liao2@student.unimelb.edu.au
%  version: Apr 2020
%% description
%  Compute the x and y displacement and velocity over time for a trajectory 
%  of several segments
%% input
%  &param tseg, is an array of size kmax X 1, containing the time duration 
%               of each segment of the trajectory.
%  &param xtCoeff, (P+1) X K_max matrix containing the coefficients
%               of the polynomials for all segments of the x(t) trajectory
%  &param ytCoeff, (P+1) X K_max matrix containing the coefficients
%               of the polynomials for all segments of the y(t) trajectory
%  &param step, plot resolution
%% output
%  &param xt,yt, x,y displacement,

%% some computations 
% total time
tTotal = sum(tSeg);
% number of plot points of the total trajectory
N = tTotal/step+1;
% number of segments
Nseg = length(tSeg);
% initialise x, y
xt = zeros(1,N);
yt = zeros(1,N);

% generating trajectory and velocity for x and y
tstart = 0;
for i=1:Nseg
    % current segment final time
    tfinal = tstart+tSeg(i);
    % current segment indices for whole traj
    idi = round(tstart/step+1);
    idf = round(tfinal/step+1);
    % current segment local time
    t = 0:0.01:tSeg(i);
    
    % polynomial expressions for x,xd,y,yd
    Xpoly = flip(xtCoeff(:,i));
    Ypoly = flip(ytCoeff(:,i));
    
    % numerical evaluation for x,xd,y,ud
    xti = polyval(Xpoly,t);
    yti = polyval(Ypoly,t);
    
    % writing to initiliased data
    xt(idi:idf) = xti;
    yt(idi:idf) = yti;
    
    % update start time for next segment
    tstart = tfinal;
end