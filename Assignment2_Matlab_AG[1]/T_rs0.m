function Trs0 = T_rs0(Tdh)
%% description
%	T_rs0 takes the M*4 X 4 transformation between subsequent frames. and 
%   computes transformation matrix between each non-inertial frame to 
%   frame 0
%% input
%  &param Tdh,  4M x 4 matrix, M number of 4 x 4 transitional matrices  
%   between subsequent joint frames
%% output
%  &param Trs0,  4M x 4 matrix, M number of 4 x 4 transitional matrices  
%   between each non-inertial frame to frame {0}

    m = length(Tdh)/4;
    Trs0 = Tdh;
    %transformation matrix of current frame to {0}
    for i = 1:m
        Tc0 = eye(4); 
        for j = 1:i
            Tc0 = Tdh(4*(i-j)+1:4*(i-j+1),1:4)*Tc0;
        end
        Tc0 = simplify(Tc0);
        Trs0 ((i-1)*4+1:i*4,1:4) = Tc0;
    end
end