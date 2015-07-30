function  out = Y_DFK_SV (dq, J_a)
%% Differential forward kinematicss
% Inputs:
%    dq = Joint velocities of the robot 
%    Ja = Analytical Jacobian matrix, where
%            Ja = [J_a(1),  J_a(4),  J_a(7), J_a(10), J_a(13), J_a(16);
%                  J_a(2),  J_a(5),  J_a(8), J_a(11), J_a(14), J_a(17);
%                  J_a(3),  J_a(6),  J_a(9), J_a(12), J_a(15), J_a(18)];
% Outputs:
%    dX = Operational velocities accordingly to the Euler parametrization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Write your code here

Ja = [J_a(1),  J_a(4),  J_a(7), J_a(10), J_a(13), J_a(16);
      J_a(2),  J_a(5),  J_a(8), J_a(11), J_a(14), J_a(17);
	  J_a(3),  J_a(6),  J_a(9), J_a(12), J_a(15), J_a(18)];
dX=Ja*dq;
%dX = zeros(6,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Outputs
%out = (diag([1;1;1;0;0;0])*Ja)*dq;
out=dX;
end 