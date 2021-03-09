function success = robot_3dplot(q1a,q2a,q3a,q4a,q5a,d1m,d2m,d3m,d4m,d5m,Trs0)
%% description
%  a 3d plot of the robot configuration given the parameters of qk and
%  defined lengths.
%% input
%  &param q1,q2,q3,q4,q5, angles
%  &param d1,d2,d3,d4,d5  length
%  &param Trs0, raw sym of transformation matrices to {0}

syms d1 d2 d3 d4 d5 d6 q1 q2 q3 q4 q5
max_dim = d1m+d3m;
%% create plot of workspace
%% choice of robot parameters
% sub in value of angles
Trs0 = subs(Trs0,{q1,q2,q3,q4,q5,d1,d2,d3,d4,d5},...
                        {q1a,q2a,q3a,q4a,q5a,d1m,d2m,d3m,d4m,d5m});
% transformation matrices from non-inertial frames to frame {0}
T20 = Trs0(5:8,1:4);
T30 = Trs0(9:12,1:4);
T40 = Trs0(13:16,1:4);
T50 = Trs0(17:20,1:4);
TE0 = Trs0(21:24,1:4);

% cylinder resolution
render_res = 20;
% radius of the cylinder
rc = 0.02; 

% a vertical cylinder about z0
[xv,yv,zv] = cylinder(rc, render_res);
% a trick to rotate the default cylinder to be about x0 by 90 about y0
Thvy = [0 0 1 0;0 -1 0 0;-1 0 0 0;0 0 0 1];
% a horizontal cylinder about x0
[xhx,yhx,zhx] = transform3d(xv, yv, zv, Thvy);
% a trick to rotate the default cylinder to be about y0 by -90 about x0
Thvx = [1 0 0 0;0 0 1 0;0 -1 0 0;0 0 0 1];
[xhy,yhy,zhy] = transform3d(xv, yv, zv, Thvx);

%% arms
x0 = xv; y0=yv; z0=zv;
z0 = z0 * d1m;
x2 = xhx; y2=yhx; z2=zhx;
x2 = x2 * d2m;
x3 = xhx; y3=yhx; z3=zhx;
x3 = x3 * d3m;
x4 = xhy; y4=yhy; z4=zhy;
y4 = y4 * (-d4m);
x5 = xv; y5=yv; z5=zv;
z5 = z5 * d5m-rc;
%% linkage joints
% link 2
xL2=xv; yL2=yv; zL2 = (zv-0.5)*rc;
% link 3
xL3=xv; yL3=yv; zL3 = (zv-0.5)*rc;
% link 4
xL4=xv; yL4=yv; zL4 = (zv-0.5)*rc;
%% end effector as a ball
[xe,ye,ze] = sphere(render_res);
xe = rc*xe; ye=rc*ye; ze=rc*ze+rc;

%% rotate to designated joint coordinates
[x2r, y2r, z2r] = transform3d(x2, y2, z2, T20);
[xL2r, yL2r, zL2r] = transform3d(xL2, yL2, zL2, T20);
[x3r, y3r, z3r] = transform3d(x3, y3, z3, T30);
[xL3r, yL3r, zL3r] = transform3d(xL3, yL3, zL3, T30);
[x4r, y4r, z4r] = transform3d(x4, y4, z4, T40);
[xL4r, yL4r, zL4r] = transform3d(xL4, yL4, zL4, T40);
[x5r, y5r, z5r] = transform3d(x5, y5, z5, T50);
[xer, yer, zer] = transform3d(xe, ye, ze, TE0);

% plotting robot in zero config
% robot base
figure
cyl0 = surf(x0,y0,z0);
set(cyl0,'FaceColor',[0 0.2 0.8],'FaceAlpha',1.0,'FaceLighting','gouraud');
hold on
cyl2 = surf(x2r,y2r,z2r);
set(cyl2,'FaceColor',[0 0.2 0.8],'FaceAlpha',1.0,'FaceLighting','gouraud');
cyl2L = surf(xL2r,yL2r,zL2r);
set(cyl2L,'FaceColor',[0.8 0.2 0], ...
       'FaceAlpha',1.0,'FaceLighting','gouraud')
cyl3 = surf(x3r,y3r,z3r);
set(cyl3,'FaceColor',[0 0.2 0.8],'FaceAlpha',1.0,'FaceLighting','gouraud');
cyl3L = surf(xL3r,yL3r,zL3r);
set(cyl3L,'FaceColor',[0.8 0.2 0],'FaceAlpha',1.0,'FaceLighting','gouraud');
cyl4 = surf(x4r,y4r,z4r);
set(cyl4,'FaceColor',[0 0.2 0.8],'FaceAlpha',1.0,'FaceLighting','gouraud');
cyl4L = surf(xL4r,yL4r,zL4r);
set(cyl4L,'FaceColor',[0.8 0.2 0],'FaceAlpha',1.0,'FaceLighting','gouraud');
cyl5 = surf(x5r,y5r,z5r);
set(cyl5,'FaceColor',[0 0.2 0.8],'FaceAlpha',1.0,'FaceLighting','gouraud');
balle = surf(xer,yer,zer);
set(balle,'FaceColor',[0.4 0.8 0.8],'FaceAlpha',1.0,'FaceLighting','gouraud');
title("robot plot")
xlabel("x_0(m)")
ylabel("y_0(m)")
zlabel("z_0(m)")
axis equal
% axis([-max_dim/2 max_dim/2 -max_dim/4 max_dim 0 max_dim]);
view(45,15);
success = 1;
end