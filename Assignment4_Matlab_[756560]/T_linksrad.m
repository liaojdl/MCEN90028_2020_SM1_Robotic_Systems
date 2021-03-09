function [Rm,Tm] = T_linksrad(DH_table)
%% description
%	T_links takes in D-H table DH_row of size M X 4 elements, and computes 
%   the 4 by 4 homogeneous translational matrix betweeen connecting joints
%% input
%  &param DH_table, M x 4 matrix
%% output
%  &param Tm,  4M x 4 matrix, M number of 4 x 4 transitional matrices  
%   between subsequent joint frames

    [m,~] = size(DH_table);
    %initialise symbolic matrix of transition matrices
    Tm = sym(zeros(4*m,4));
    %initialise symbolic matrix of rotation matrices
    Rm = sym(zeros(3*m,3));
    for i=1:m
        [r_link,t_link] = T_linkrad(DH_table(i,:));
        Tm((4*(i-1)+1):4*i,:) = t_link;
        Rm((3*(i-1)+1):3*i,:) = r_link;
    end
        
        
end

