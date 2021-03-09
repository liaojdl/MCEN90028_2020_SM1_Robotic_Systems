function J = calculate_Jacobian(Q)
%% description
%	calculate_Jacobian takes in values of Q, [q1,q2,q3,q4,q5]
%% input
%  &param Q [q1,q2,q3,q4,q5]
%% output
%  &param J, 6 X 5 jacobian numerical output

% %DH Table
% Mdh = [0 0 d1 (q1+pi/2);
%         0 pi/2 0 q2;
%         d2 0 0 q3;
%         d3 0 0 (q4+pi/2);
%         0 pi/2 0 q5;
%         0 pi -d4 0]; 

% joint rotations
q1 = Q(1);
q2 = Q(2);
q3 = Q(3);
q4 = Q(4);
q5 = Q(5);

% dimensions
d1 = 0.15;
d2 = 0.15;
d3 = 0.2;
d4 = 0.15;


J = ...
[-cos(q1)*(d3*cos(q2 + q3) + d2*cos(q2) + d4*cos(q2 + q3 + q4)),   sin(q1)*(d3*sin(q2 + q3) + d2*sin(q2)) + d4*sin(q2 + q3 + q4)*sin(q1),  sin(q1)*(d3*sin(q2 + q3) + d4*sin(q2 + q3 + q4)),  d4*sin(q2 + q3 + q4)*sin(q1),                          0;
 -sin(q1)*(d3*cos(q2 + q3) + d2*cos(q2) + d4*cos(q2 + q3 + q4)), - cos(q1)*(d3*sin(q2 + q3) + d2*sin(q2)) - d4*sin(q2 + q3 + q4)*cos(q1), -cos(q1)*(d3*sin(q2 + q3) + d4*sin(q2 + q3 + q4)), -d4*sin(q2 + q3 + q4)*cos(q1),                          0;
                                                              0,                     d3*cos(q2 + q3) + d2*cos(q2) + d4*cos(q2 + q3 + q4),            d3*cos(q2 + q3) + d4*cos(q2 + q3 + q4),          d4*cos(q2 + q3 + q4),                          0;
                                                              0,                                                                 cos(q1),                                           cos(q1),                       cos(q1), -cos(q2 + q3 + q4)*sin(q1);
                                                              0,                                                                 sin(q1),                                           sin(q1),                       sin(q1),  cos(q2 + q3 + q4)*cos(q1);
                                                              1,                                                                       0,                                                 0,                             0,          sin(q2 + q3 + q4)];
 
end

