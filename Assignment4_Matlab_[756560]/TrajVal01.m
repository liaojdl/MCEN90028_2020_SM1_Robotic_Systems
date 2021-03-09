function xt = TrajVal01(tSeg,xtCoeff,step)
%  author: Jiawei Liao 756560 liao2@student.unimelb.edu.au
%  version: May 2020
%% description
%  Compute the values for a trajectory of several segments
%% input
%  &param tseg, is an array of size kmax X 1, containing the time duration 
%               of each segment of the trajectory.
%  &param xtCoeff, (P+1) X K_max matrix containing the coefficients
%               of the polynomials for all segments of the x(t) trajectory
%  &param step, resolution
%% output
%  &param xt,   polynomial value of trajectory evaluated at time steps

%% some computations 
% total time
tTotal = sum(tSeg);
% number of plot points of the total trajectory
N = round(tTotal/step)+1;
% number of segments
Nseg = length(tSeg);
% initialise x, y
xt = zeros(1,N);
% generating trajectory and velocity for x and y
tstart = 0;
for i=1:Nseg
    % current segment final time
    tfinal = tstart+tSeg(i);
    % current segment indices for whole traj
    idi = round(tstart/step)+1;
    idf = round(tfinal/step)+1;
    % current segment local time
    t = 0:step:tSeg(i);
    % polynomial expressions for x
    Xpoly = flip(xtCoeff(:,i));
    % numerical evaluation for x
    xti = polyval(Xpoly,t);
    % writing to initiliased data
    xt(idi:idf) = xti;
    % update start time for next segment
    tstart = tfinal;
end