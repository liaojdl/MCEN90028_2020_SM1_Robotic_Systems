function dydt = Task3_dynamics(t,y, tau)
    
    % hard limits on joint 1
    if (y(1) < (-60*pi/180))
        y(1) = -60*pi/180;
        y(3) = 0;
    elseif (y(1) > (60*pi/180))
        y(1) = 60*pi/180;
        y(3) = 0;
    end
    % hard limits on joint 2
    if (y(2) < (-90*pi/180))
        y(2) = -90*pi/180;
        y(4) = 0;
    elseif (y(2) > (90*pi/180))
        y(2) = 90*pi/180;
        y(4) = 0;
    end
    % x represents the state that contains q and qdot for the robot 
    % ie. for both joints
    q = [y(1); y(2)];
    qdot = [y(3); y(4)];
    q1 = y(1);
    q2 = y(2); 
    q1dot = y(3);
    q2dot = y(4);

    m1 = 2; %(kg)
    m2 = 1; %(kg)
    Izz1 = 0.5; 
    Izz2 = 0.3;
    Ixx1 = 0;
    Iyy1 = 0;
    Ixx2 = 0;
    Iyy2 = 0;
    L1 = 1.0;
    L2 = 0.6;
    rc1 = 0.5; 
    rc2 = 0.3; 
    g = 9.8;

    % inertia matrix
    A = [m2*L1^2 + 2*m2*cos(q2)*L1*rc2 + m1*rc1^2 + m2*rc2^2 + Izz1 + Izz2, m2*rc2^2 + L1*m2*cos(q2)*rc2 + Izz2; m2*rc2^2 + L1*m2*cos(q2)*rc2 + Izz2, m2*rc2^2 + Izz2];
    % Coriolis and  centrifugal
    B = [-2*L1*m2*rc2*sin(q2); 0];
    C = [0, -L1*m2*rc2*sin(q2); L1*m2*rc2*sin(q2), 0];

    % gravity term
    G = [g*m2*(rc2*cos(q1 + q2) + L1*cos(q1)) + g*m1*rc1*cos(q1); g*m2*rc2*cos(q1 + q2)];

    % friction
    friction_coeff = 0.1;
    friction = friction_coeff * [q1dot ; q2dot];
    
    %Calculating the qdoubledot -- joint space acceleration
    qddot = inv(A) * (tau - friction - B * (q1dot *q2dot) - C * [q1dot^2; q2dot^2] - G); % this is qdoubledot.

    dydt = zeros(4,1);
    dydt(1) = y(3);
    dydt(2) = y(4);
    dydt(3) = qddot(1);
    dydt(4) = qddot(2);
    
end