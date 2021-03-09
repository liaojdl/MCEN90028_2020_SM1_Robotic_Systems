function dydt = Task2_dynamics2(t,y,tau)
    % x represents the state that contains q and qdot for the robot 
    % ie. for both joints
    q = [y(1); y(2)];
    qdot = [y(3); y(4)];
    q1 = y(1);
    q2 = y(2);
    q1dot = y(3);
    q2dot = y(4);
    
    % Define some system parameters
    m1 = 2; %(kg)
    m2 = 1; %(kg)
    L1 = 1;
    L2 = 0.6;
    Izz1 = 0.5; 
    Izz2 = 0.3;
    rc1 = 0.5; % (m) distance to center of mass 1
    rc2 = 0.3; % (m) distance to center of mass 2
    g = 9.8;

    % Inertia matrix
    A = [m2*L1^2 + 2*m2*cos(q2)*L1*rc2 + m1*rc1^2 + Izz1 + Izz2 + m2*rc2^2, m2*rc2^2 + L1*m2*cos(q2)*rc2 + Izz2;
        m2*rc2^2 + L1*m2*cos(q2)*rc2 + Izz2, m2*rc2^2 + Izz2];
    
    % Coriolis and  centrifugal
    B = [-2*L1*m2*rc2*sin(q2); 0];
    C = [0, -L1*m2*rc2*sin(q2); L1*m2*rc2*sin(q2), 0];
    
    % Gravity
    G = [g*m2*(rc2*cos(q1+q2) + L1*cos(q1)) + g*m1*rc1*cos(q1);
            g*m2*rc2*cos(q1+q2)];
        
    % Gravity Compensation
%     tau = G; % Task 1 Part 2
%     m1x = 2; % Wrong estimated value of mass 1
%     tau = [g*m2*(rc2*cos(q1+q2) + L2*cos(q1)) + g*m1x*rc1*cos(q1);
%             g*m2*rc2*cos(q1+q2)];
    
    % friction
    friction_coeff = 0.1;
    friction = friction_coeff * [q1dot ; q2dot];
    
    % Calculating the qdoubledot -- joint space acceleration
    qddot = inv(A) * (tau - friction - B * (q1dot*q2dot) - C * [q1dot^2; q2dot^2] - G); % this is qdoubledot.

    dydt = zeros(4,1);
    dydt(1) = y(3);
    dydt(2) = y(4);
    dydt(3) = qddot(1);
    dydt(4) = qddot(2);
    
end