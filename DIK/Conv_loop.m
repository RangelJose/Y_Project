function out = Conv_loop(J_R,J_L,d_X_R,d_X_L)
%% Priority loop and Convergence loop
% Inputs:
%    J_R = Right end-effector Jacobian
%    J_L = Left end-effector Jacobian
%    d_X_R = Right end-effector delta x
%    d_X_L = Left end-effector delta x
%       Where
%       JR = [J_R(1) J_R(4) J_R(7) J_R(10) J_R(13) J_R(16);
%             J_R(2) J_R(5) J_R(8) J_R(11) J_R(14) J_R(17);
%             J_R(3) J_R(6) J_R(9) J_R(12) J_R(15) J_R(18)];
%       JL = [J_L(1) J_L(4) J_L(7) J_L(10) J_L(13) J_L(16);
%             J_L(2) J_L(5) J_L(8) J_L(11) J_L(14) J_L(17);
%             J_L(3) J_L(6) J_L(9) J_L(12) J_L(15) J_L(18)];
%       d_X_R=[X;
%              Y;
%              E2];
%       d_X_L=[X;
%              Y;
%              E2];
% Outputs:
%    J = Jacobian matrix for both end-effectors
%    d_X{1} = Delta x for the R_Task and the right end-effector
%    d_X{2} = Delta x for the L_Task and the left end-effector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p = 3; %Priorities
d_X = cell(p,1);
J = cell(p,1);
Jt = cell(p,1);
d_T = cell(p,1);
Pn = cell(p,1);

%Initial Conditions
T=0.001;
Pn{1}=eye(6);
d_T{1}=[0;0;0;0;0;0];
%

J{2} = [J_R(1) J_R(4) J_R(7) J_R(10) J_R(13) J_R(16);
        J_R(2) J_R(5) J_R(8) J_R(11) J_R(14) J_R(17);
        J_R(3) J_R(6) J_R(9) J_R(12) J_R(15) J_R(18)];
J{3} = [J_L(1) J_L(4) J_L(7) J_L(10) J_L(13) J_L(16);
        J_L(2) J_L(5) J_L(8) J_L(11) J_L(14) J_L(17);
        J_L(3) J_L(6) J_L(9) J_L(12) J_L(15) J_L(18)];

d_X{2} = [d_X_R(1);d_X_R(2);d_X_R(3)];
d_X{3} = [d_X_L(1);d_X_L(2);d_X_L(3)];

for i=2:3
d_X{i} = d_X{i}-(J{i}*d_T{i-1});
J{i} = J{i}*Pn{i-1};
Jt{i} = pinv(J{i});
d_T{i} = d_T{i-1}+(Jt{i}*d_X{i});
Pn{i} = Pn{i-1}-(Jt{i}*J{i});
end
% 
% d_X{2}=d_X{2}-(J{2}*d_T{1});
% J{2}=J{2}*Pn{1};
% Jt{2}=pinv(J{2});
% d_T{2}=d_T{1}+(Jt{2}*d_X{2});
% Pn{2}=Pn{1}-(Jt{2}*J{2});
% 
% d_X{3}=d_X{3}-(J{3}*d_T{2});
% J{3}=J{3}*Pn{2};
% Jt{3}=pinv(J{3});
% d_T{3}=d_T{2}+(Jt{3}*d_X{3});
% Pn{3}=Pn{2}-(Jt{3}*J{3});

dq=d_T{i}/T;
out =dq;
end