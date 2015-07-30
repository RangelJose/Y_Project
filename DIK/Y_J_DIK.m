function out = Y_J_DIK(q,R_Task,L_Task)
%% Jacobian Matrix
% Inputs:
%    q = Robot configuration, i.e. each element of q is a joint position
%    R_Task = Task to be accomplished by the right end-effector
%    L_Task = Task to be accomplished by the left end-effector
%       Where
%       Task= [X;
%              Y;
%              E2]
% Outputs:
%    J = Jacobian matrix for both end-effectors
%    d_X{1} = Delta x for the R_Task and the right end-effector
%    d_X{2} = Delta x for the L_Task and the left end-effector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global srcLoaded Ymodel EulerConvention
if isempty(Ymodel)
    if isempty(srcLoaded)
        addpath(genpath('src'));
        display('--> Folder src and subfolders added to the path')
        srcLoaded = true;
    end
    Y_Model();
    Euler_Convention(1);
    display('--> Y model loaded')
end


%Starting Forward Kinematics
T=body_LF_Transform (Ymodel.NB, Ymodel.LFinPF, Ymodel.axes, q, Ymodel.parent);
T_ee{1} = T{5}*Ymodel.EEinLF{1}; %End-effector Right Arm
T_ee{2} = T{6}*Ymodel.EEinLF{2}; %End-effector Left Arm
% Forward Kinematics

% Obtaining end-effector final position
O_n{1} = T_ee{1}(1:3,4);
O_n{2} = T_ee{2}(1:3,4);
% End-effector final position obtained

% Obtaining end-effector final orientation
[~,Ey{1},~]=Conventions(EulerConvention.IndexSaved,T_ee{1}(1:3,1:3));
[~,Ey{2},~]=Conventions(EulerConvention.IndexSaved,T_ee{2}(1:3,1:3));
% End-effector final orientation Obtained

% Obtaining geometric Jacobian
[Jv{1},Jw{1}] = Jg_EE(Ymodel.NB,O_n{1},T,5,Ymodel.parent,Ymodel.axes);
[Jv{2},Jw{2}] = Jg_EE(Ymodel.NB,O_n{2},T,6,Ymodel.parent,Ymodel.axes);
% Geometric Jacobian Obtained

% Obtaining B Matrix
B{1} = BMatrix(EulerConvention.axes,0,Ey{1},0);
B{2} = BMatrix(EulerConvention.axes,0,Ey{2},0);
% B Matrix Obtained

% Compensating the B Matrix
B{1}=B{1}+(eye(3)*0.001);
B{2}=B{2}+(eye(3)*0.001);
% B Matrix Compensated

% Compensating euler velocity Jacobian
Jw{1}=B{1}\Jw{1};
Jw{2}=B{2}\Jw{2};
% Euler velocity Jacobian Compensated

% Obtaining euler velocity Jacobian
Jwb{1} = Jw{1}(2,:);
Jwb{2} = Jw{2}(2,:);
% Euler velocity Jacobian

% Building Jacobian
J{1} = [Jv{1}(1,:);Jv{1}(3,:);Jwb{1}];
J{2} = [Jv{2}(1,:);Jv{2}(3,:);Jwb{2}];
% Jacobian Builded

% Obtaining Delta X
d_X{1} = [R_Task(1)-O_n{1}(1);R_Task(2)-O_n{1}(3);R_Task(3)-Ey{1}]; %D_X = Xd-X
d_X{2} = [L_Task(1)-O_n{2}(1);L_Task(2)-O_n{2}(3);L_Task(3)-Ey{2}];
d_X{1}
% Delta X obtained


out=[J{1}(:,1);J{1}(:,2);J{1}(:,3);J{1}(:,4);J{1}(:,5);J{1}(:,6);
     J{2}(:,1);J{2}(:,2);J{2}(:,3);J{2}(:,4);J{2}(:,5);J{2}(:,6);
     d_X{1};d_X{2}];