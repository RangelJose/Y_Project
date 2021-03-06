function  B = Bxyz (e)
%% B Matrix
% Inputs:
%    e = Euler angles accordingly to the XYZ convention
% Outputs:
%    B = A 3x3 matrix such that Ja_e = inv(B)*Jg_w
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Re1 = Rx(e(1)); 
Re2 = Ry(e(2)); 
Re3 = Rz(e(3)); 
R1 = Re1;
R2 = R1 * Re2;
R3 = R2 * Re3;
B = [R1(:,1) R2(:,2) R3(:,3)];

end