% MCEN9028 Robotics System Assignment 4 task 1
% Deriving the EOM for 2-d planar revoute-revolute robot
% Version 1.0 2020 May, Jiawei Liao, 756560,
% mailto:liao2@student.unimelb.edu.au

%% Full clean up
clc; clear all; close all;

%% Some definitions
syms Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 rc1 rc2 L1 L2 q1 q1d q1dd q2 q2d q2dd
%angular moment of intertia for links
Ic1 = [Ixx1,0,0;0,Iyy1,0;0,0,Izz1];
Ic2 = [Ixx2,0,0;0,Iyy2,0;0,0,Izz2];
%angular velocities between frames
W10 = [0;0;q1d];
W21 = [0;0;q2d];
%DOF
N = 2;

%% the DH table
%DH Table
Mdh = [0 0 0 q1;
       L1 0 0 q2;
       L2 0 0 pi/2;
       0 pi/2 0 0];
%transformation matrices between subsequent frames
[Rdh,Tdh] = T_linksrad(Mdh);
%transfomration matrices from all non-inertial frames to frame 0
[R_rs0,T_rs0] = T_rs0(Tdh)
%extract some matrices
T10 = T_rs0(1:4,1:4);
T20 = T_rs0(5:8,1:4);
R10 = R_rs0(1:3,1:3);
R20 = R_rs0(4:6,1:3);

%% Calculate the Jacobian terms at com of m1 and com of m2
%Translation velocity component
P02 = T20*[0;0;0;1]; P02 = P02(1:3);
P01 = T10*[0;0;0;1]; P01 = P01(1:3);
%The Z axis
oZ2 = R20*[0;0;1];
oZ1 = R10*[0;0;1];
Pom1 = T10*[rc1;0;0;1]; Pom1 = Pom1(1:3);
Pom2 = T20*[rc2;0;0;1]; Pom2 = Pom2(1:3);
% Pom2 = T20*[0;-rc2;0;1]; Pom2 = Pom2(1:3);

JV_m1 = simplify([diff(Pom1,q1),diff(Pom1,q2)])
JV_m2 = simplify([diff(Pom2,q1),diff(Pom2,q2)])

%angular velocity component are the same for both com
W20 = R10*W10 + R20*W21;
JW_m1 = [diff(W10,q1d),diff(W10,q2d)]
JW_m2 = [diff(W20,q1d),diff(W20,q2d)]


%% Calculate matrix A the inertial terms
syms m1 m2
A = simplify(m1*(JV_m1).'*(JV_m1) + (JW_m1).'*Ic1*(JW_m1) + ...
    m2*(JV_m2).'*(JV_m2) + (JW_m2).'*Ic2*(JW_m2))


%% Calculate matrix B the coriolis terms
%The two Qs
Qk = [q1,q2];

Aijk = sym(zeros(N,N,N));
for i = 1:N
    for j = 1:N
        for k = 1:N
            Aijk(i,j,k) = diff(A(i,j),Qk(k));
        end
    end
end
Aijk;
%B has dimension N*N(N-1)/2
B = sym(zeros(N,N(N-1)/2));
for i = 1:N
    for j = 1:N-1
            p = 0;
        for k = j+1:N
            p = p+1;
            B(i,p)=(Aijk(i,j,k)+Aijk(i,k,j)-Aijk(j,k,i))/2;
        end
    end
end
B = B*2


%% Calculate matrix C the centrifugal terms
% dimension N by N
Cq = sym(zeros(N,N));
for i = 1:N
    for j = 1:N
        Cq(i,j) = (Aijk(i,j,j)+Aijk(i,j,j)-Aijk(j,j,i))/2;
    end
end
Cq


%% Calculate matrix G the gravitational terms
syms g
% gravity is in y axis direction for this case
Zg = [0;-1;0];
% component for each mass
G1 = (JV_m1.')*(-m1*g*Zg);
G2 = (JV_m2.')*(-m2*g*Zg);
Gq = simplify(G1+G2)

%% Get equations of Motion
% Form is A [qdd] + B*[qd qd] + C*[qd^2] + G(q)
syms tau1 tau2
EOM = [tau1;tau2] == A * [q1dd;q2dd] + B *[q1d*q2d] + Cq*[q1d^2;q2d^2] + Gq;
EOM = simplify(EOM)
subs (EOM, {m1,m2,Izz1,Izz2,Ixx1,Ixx2,Iyy1,Iyy2,L1,L2,rc1,rc2},...
        {2,2,0.5,0.3,0,0,0,0,1,0.6,0.5,0.3})
    
%% add in the values of parameters
m1 = 2; %(kg)
m2 = 1; %(kg)
Izz1 = 0.5;
Izz2 = 0.3;
Ixx1 = 0;
Iyy1 = 0;
Ixx2 = 0;
Iyy2 = 0;
L1 = 1;
L2 = 0.6;
rc1 = 0.5;
rc2 = 0.3;
g = 9.8;


%% Simulation of Part 2 and Part 3
% for part 3, just makes tau equal to the G matrix

dt = 0.01; 
tf = 5; 

% Initialise the robot to the initial position and velocity
q(1)= 0/180*pi; % Converting degree to radian
q(2) = 0/180*pi; 
qdot(1) = 0;
qdot(2) = 0;
i = 1;

for time=0:dt:tf
    % This is the command torque
    % zero driving torque for part 2
    tau = [0;0]; 
    % this is the same as the G matrix. for part 3 specific, uncomment for
    % use
    tau = [g*m2*(rc2*cos(q(1) + q(2)) + L1*cos(q(1))) + g*m1*rc1*cos(q(1)); g*m2*rc2*cos(q(1) + q(2))];
    [t,y] = ode45(@(t,y) Task1_dynamics(t,y,tau), [0, dt], [q(1), q(2), qdot(1), qdot(2)]);
    Ly = length(y(:,1));
    q(1) = y(Ly,1);
    q(2) = y(Ly,2);
    qdot(1) = y(Ly,3);
    qdot(2) = y(Ly,4);

    %%% Storing q and qdot values into an array
    q1s(i) = q(1);
    q2s(i) = q(2);
    qdot1s(i) = qdot(1);
    qdot2s(i) = qdot(2);
    i = i+1;

end

figure
time = 0:dt:tf;
plot(time,q1s,'-o', time,q2s,'-o')
grid on
grid minor
xlabel ('time (s)')
ylabel ('joint angle displacement (rad)')
title ('q_1,q_2 vs t')
legend('q_1','q_2')


%% animating the robot
%robot angle displacment over time
p1x = L1*cos(q1s);
p1y = L1*sin(q1s);
p2x = L2*cos(q1s + q2s)+ L1*cos(q1s);
p2y = L2*sin(q1s + q2s)+ L1*sin(q1s);

figure
grid on
hold on
axis equal
axis([-2 2 -2 0.5])
h = animatedline('MaximumNumPoints',2,'Linewidth',5,'Color',[0, 0.4470, 0.7410]);
g = animatedline('MaximumNumPoints',2,'Linewidth',5,'Color',[0.8500, 0.3250, 0.0980]);
[A,map] = rgb2ind(frame2im(getframe),512);
imwrite(A,map,'task1.gif','LoopCount',65535,'DelayTime',0.01);

for i=1:length(q1s)
    addpoints(h,0,0);
    addpoints(h,p1x(i),p1y(i));
    addpoints(g,p1x(i),p1y(i));
    addpoints(g,p2x(i),p2y(i));
    drawnow
    if(mod(i,5)==0)
        [A,map] = rgb2ind(frame2im(getframe),512);
        imwrite(A,map,'task1.gif','WriteMode','append','DelayTime',0.05);
    end
end



