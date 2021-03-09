function [Q1,Q2] = IK_robot(vf0,d1,d2,d3,d4,d5)
%% description
%	IK_robot takes in the desired end-effector coordinates in frame {0} and
%	solves for the corresponding joint angles q1 to q5 in degrees
%% input
%  &param x,y,z: task space coordinates in frame {0}

%% output
%  &param Qk, 5x1, solutions corresponding to the coordinates required
    
    %% Frame 0 working
    %Something to make things easier to read, define the position of joint 
    %4, point C of triangle for easier readability
    cx = vf0(1);cy = vf0(2);cz = vf0(3)+d4+d5;
    
    
    %% solution 1
    %the base rotational angle is easy
    q1 = atand(-cx/cy);
    %length of beam AC, or joint 2 to joint 4
    dAC = sqrt(cx^2+cy^2+(cz-d1)^2);
    %compute q3 via cosine rule, note that two possible solutions
    %exist
    num = dAC^2 - d2^2 - d3^2;
    den = 2 * d2 * d3;
    q3a = acosd(num/den);
    
    %sine rule to find angle <bac
    alpha_a = asind(d3*sind(q3a)/dAC);
    q2a = atand((cz - d1)/sqrt(cx^2 + cy^2)) - alpha_a;
    %q4 to follow always point down constraint
    q4a = -90-q3a-q2a;
    %irrelevant
    q5 = 0;
    
    Q1 = [q1;q2a;q3a;q4a;q5];
    
    %% solution 2
    q3b = -q3a;
    alpha_b = asind(d3*sind(q3b)/dAC);
    q2b = atand((cz - d1)/sqrt(cx^2 + cy^2)) - alpha_b;
    q4b = -90-q3b-q2b;
     
    Q2 = [q1;q2b;q3b;q4b;q5];
end