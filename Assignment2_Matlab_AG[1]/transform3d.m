function [xb,yb,zb] = transform3d(xa,ya,za,T)
%% description
%	transformed3d takes the 4x4 homogeneous transformation matrix and a 
%   3d co-ordinate and computes the resulting co-ordinates
%% input
%  &param xa,ya,za, co-ordinates described by frame a
%  &param T,        transformation from frame a to frame b
%% output
%  &param xb,yb,zb, co-ordinates descrived by frame b

    %initialise output to correct size
    I=size(xa,1);
    J=size(xa,2);
    xb=zeros(I,J);
    yb=zeros(I,J);
    zb=zeros(I,J);

    for ii=1:I
        for jj=1:J
            vector=[xa(ii,jj);ya(ii,jj);za(ii,jj);1];
            vector=T*vector;
                xb(ii,jj)=vector(1);
                yb(ii,jj)=vector(2);
                zb(ii,jj)=vector(3);
        end
    end        
end
