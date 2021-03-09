function [xvt,yvt] = Getxv_yv(tSeg,xtCoeff,ytCoeff,step)
%  author: Jiawei Liao 756560 liao2@student.unimelb.edu.au
%  version: Apr 2020
%% description
%  plot the x and y displacement and velocity over time for a trajectory 
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
%  &param xt,yt,xvt,yvt, x,y displacement, x,y velocity

%% some computations 
% total time
tTotal = sum(tSeg);
% number of plot points of the total trajectory
N = tTotal/step+1;
% number of segments
Nseg = length(tSeg);
% initialise x, xd, y, yd
xvt = zeros(1,N);
yvt = zeros(1,N);

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
    Xvpoly = polyder(Xpoly);
    Ypoly = flip(ytCoeff(:,i));
    Yvpoly = polyder(Ypoly);
    
    % numerical evaluation for x,xd,y,ud
    xvti = polyval(Xvpoly,t);
    yvti = polyval(Yvpoly,t);
    
    % writing to initiliased data
    xvt(idi:idf) = xvti;
    yvt(idi:idf) = yvti;
    
    % update start time for next segment
    tstart = tfinal;
end