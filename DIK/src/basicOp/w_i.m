function wi = w_i(axis, qp)
%% Angular velocity of link i relative to frame i-1 
% Inputs: 
%    axis = axis in frame i-1 where the joint i acts
%    qp = joint velocity of link i
%
% Outputs: 
%    wi = Angular velocity of link i produced by q(i) expressed in frame i-1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch axis
    case 1
        wi = [qp; 0; 0];
    case 2
        wi = [0; qp; 0];
    case 3
        wi = [0; 0; qp];
    otherwise
        wi = [0; 0; 0];
end
end