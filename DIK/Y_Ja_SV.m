function  out = Y_Ja_SV (J_g)
%% Analytical Jacobian matrix
% Inputs:
%    Jg_RH = Geometric Jacobian matrix, where
%            Jg = [J_g(1),  J_g(4),  J_g(7), J_g(10), J_g(13), J_g(16);
%                  J_g(2),  J_g(5),  J_g(8), J_g(11), J_g(14), J_g(17);
%                  J_g(3),  J_g(6),  J_g(9), J_g(12), J_g(15), J_g(18)];
%                  
%       EE_Pose = [J_g(19)
%                  J_g(20)
%                  J_g(21)];
%
%    q = Robot configuration, i.e. each element of q is a joint position
% Outputs:
%    Ja = Analaytical Jacobian matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global EulerConvention
Jv = [J_g(1),  J_g(4),  J_g(7), J_g(10), J_g(13), J_g(16);
	  J_g(2),  J_g(5),  J_g(8), J_g(11), J_g(14), J_g(17)];

Jw = [0,0,0,0,0,0;
    J_g(3),  J_g(6),  J_g(9), J_g(12), J_g(15), J_g(18);
    0,0,0,0,0,0];
%Jw = [J_g(4),  J_g(10), J_g(16), J_g(22), J_g(28), J_g(34);
% 	   J_g(5),  J_g(11), J_g(17), J_g(23), J_g(29), J_g(35);
%      J_g(6),  J_g(12), J_g(18), J_g(24), J_g(30), J_g(36)];

B=BMatrix(EulerConvention.axes,0,J_g(21),0);
B=B+(eye(3)*0.001);
Jwa=inv(B)*Jw;

Ja=[Jv;Jwa(2,:)];
%Ja=zeros(6,24);
out = [Ja(:,1); Ja(:,2); Ja(:,3); Ja(:,4); Ja(:,5); Ja(:,6)];