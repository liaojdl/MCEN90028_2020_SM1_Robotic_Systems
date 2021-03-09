function Success = robot_simplews_plot(q1L,q2L,q3L,q4L,q5L,d1m,d2m,d3m,d4m,d5m,res)
%% description
%  plots the workspace of the specific 5-DOF robot designed 
%  for picking up the jenga tower.
%  two views are given: top view, cross section view in frame {1}
%  q5 is also irrelvant as it only orientates the gripper -90 to 90 degrees

    %simple rotation
    R2d = @(x)[cosd(x) -sind(x);
        sind(x) cosd(x)];

    q1span = linspace(q1L(1),q1L(2),res);
    q2span = linspace(q2L(1),q2L(2),res);
    q3span = linspace(q3L(1),q3L(2),res);
    q4span = linspace(q4L(1),q4L(2),res);
    L1 = length(q1span);
    L2 = length(q2span);
    L3 = length(q3span);
    L4 = length(q4span);
    max_dim = d1m+d2m+d3m+d4m+d5m;


    %% Cross-sectional view plot
    %initialize output
    xc = zeros(L2*L3*L4,1);
    yc = zeros(L2*L3*L4,1);
    vc = zeros(L2*L3*L4,1);
    for i=1:L2
        for j=1:L3
            for k=1:L4
                coor = R2d(q4span(k))*[d4m+d5m;0];
                coor = R2d(q3span(j))*(coor+[d3m;0]);
                coor = R2d(q2span(i))*(coor+[d2m;0]);
                coor = coor+[0;d1m];
                xc(k+(j-1)*L3+(i-1)*L4*L3) = coor(1);
                yc(k+(j-1)*L3+(i-1)*L4*L3) = coor(2);
            end
        end
    end

    offsetxs = 0.15;
    jengatower_side = polyshape([offsetxs offsetxs 0.075+offsetxs 0.075+offsetxs],...
                            [0.27 0 0 0.27]);

    figure
    title("Cross-Sectional View of Robot Workspace in Frame \{1\}")
    xlabel("x_1(m)")
    ylabel("z_1(m)")
    hold on
    grid on
    scatter3(xc,yc,vc,'MarkerEdgeColor',[0.3010 0.7450 0.9330]);
    plot(jengatower_side,'FaceColor',[1 0 0]);
    % scatter3(xc,yc,vc,'MarkerEdgeColor','k','MarkerFaceColor',[0.2 .45 .45]);
    axis equal
    axis([-max_dim/2 max_dim -max_dim/2 max_dim -max_dim max_dim]);
    hold off

    %% top down view plot
    
    offsetxt = -0.075/2;
    offsetyt = offsetxs;
    jengatower_top = polyshape([0 0 0.075 0.075]+offsetxt,...
                            [0.075 0 0 0.075]+offsetyt);
                        
    max_r = max(xc);
    y_circle = zeros(L1,1);
    x_circle = zeros(L1,1);
    for i=1:L1
        y_circle(i) = max_r*cosd(q1span(i));
        x_circle(i) = max_r*sind(q1span(i));
    end
    figure
    hold on
    opcircle = polyshape(x_circle,y_circle);
    plot(opcircle,'FaceColor',[0.3010 0.7450 0.9330])
    plot(jengatower_top,'FaceColor',[1 0 0]);
    axis equal
    axis([-max_dim max_dim -max_dim max_dim]);
    title("Top View of Robot Workspace in Frame \{0\}")
    xlabel("x_0(m)")
    ylabel("y_0(m)")
    grid on
    hold off;

    Success = 1;
end