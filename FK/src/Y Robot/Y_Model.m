function Y_Model()
%% Bioloid Model
%In this section is where all the parameters of the robot are controlled
% NB is the number of DOF of the robot
% link is the number of the link of every joint
% axes is the axis where every joint moves
% 1= Rotation in X
% 2= Rotation in Y
% 3= Rotation in Z
% 4= Traslation in X
% 5= Traslation in Y
% 6= Traslation in Z
% parent is the parent of every joint, it means the transformation before
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global srcLoaded Ymodel
if isempty(srcLoaded)
    addpath(genpath('../../src'));
    display('--> Folder src and subfolders added to the path')
    srcLoaded = true;
end

Ymodel.NB = 6;
Ymodel.link =   [ 1 2 3 4 5 6 ];%1to6 referential frame to Body frame, 7to9 Right Arm, 10to12 Left Arm, 13to18 Right Leg, 19to24 Left Leg
Ymodel.axes =   [ 2 2 2 2 2 2 ];
Ymodel.parent = [ 0 1 2 2 3 4 ];

% S = cell(RJ1model.NB,1);
% for i = 1 : RJ1model.NB
%     switch  RJ1model.axes(i)
%         case 1
%             S{i}=[1; 0; 0; 0; 0; 0];
%         case 2
%             S{i}=[0; 1; 0; 0; 0; 0];
%         case 3
%             S{i}=[0; 0; 1; 0; 0; 0];
%         case 4
%             S{i}=[0; 0; 0; 1; 0; 0];
%         case 5
%             S{i}=[0; 0; 0; 0; 1; 0];
%         case 6
%             S{i}=[0; 0; 0; 0; 0; 1];
%     end         
% end

%% Joint limits
Ymodel.jntVelLim = [ inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf];
Ymodel.jntLim = [ inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf];

%% Position of the referential frame of each body in the parent coordinates
%"Variable" distances between joints

Ymodel.LFinPF{1} = Tras([0,0,0]);
Ymodel.LFinPF{2} = Tras([0,0,2]);
Ymodel.LFinPF{3} = Tras([0,0,2]);
Ymodel.LFinPF{4} = Tras([0,0,2]);
Ymodel.LFinPF{5} = RotY(.7854)*Tras([0,0,2]);
Ymodel.LFinPF{6} = RotY(-.7854)*Tras([0,0,2]);

%% Position of the referential frame of the end-effector in the body local frame
Ymodel.EEinLF{1} = Tras([0,0,2])*RotY(-.7854);%End-effector Right hand
Ymodel.EEinLF{2} = Tras([0,0,2])*RotY(.7854);%End-effector Left hand

%% Position of the wrist wrt the end-efector frame
%Ymodel.eePw = -Ymodel.EEinLF{1}(1:3,1:3).'*(Ymodel.EEinLF{1}(1:3,4) + [0; 0; l]);
%It hasn't been designed due to we are not using it, it has no changes from
%the original.
%% dynamic parameters
Ymodel.m{1}=0;
Ymodel.CoM{1}=[0; 0; 0];
Ymodel.I{1} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{2}=0;
Ymodel.CoM{2}=[0; 0; 0];
Ymodel.I{2} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{3}=0;
Ymodel.CoM{3}=[0; 0; 0];
Ymodel.I{3} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{4}=0;
Ymodel.CoM{4}=[0; 0; 0];
Ymodel.I{4} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{5}=0;
Ymodel.CoM{5}=[0; 0; 0];
Ymodel.I{5} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{6}=0;
Ymodel.CoM{6}=[0; 0; 0];
Ymodel.I{6} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{7}=0;
Ymodel.CoM{7}=[0; 0; 0];
Ymodel.I{7} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{8}=0;
Ymodel.CoM{8}=[0; 0; 0];
Ymodel.I{8} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{9}=0;
Ymodel.CoM{9}=[0; 0; 0];
Ymodel.I{9} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{10}=0;
Ymodel.CoM{10}=[0; 0; 0];
Ymodel.I{10} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{11}=0;
Ymodel.CoM{11}=[0; 0; 0];
Ymodel.I{11} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{12}=0;
Ymodel.CoM{12}=[0; 0; 0];
Ymodel.I{12} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{13}=0;
Ymodel.CoM{13}=[0; 0; 0];
Ymodel.I{13} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{14}=0;
Ymodel.CoM{14}=[0; 0; 0];
Ymodel.I{14} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{15}=0;
Ymodel.CoM{15}=[0; 0; 0];
Ymodel.I{15} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{16}=0;
Ymodel.CoM{16}=[0; 0; 0];
Ymodel.I{16} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{17}=0;
Ymodel.CoM{17}=[0; 0; 0];
Ymodel.I{17} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{18}=0;
Ymodel.CoM{18}=[0; 0; 0];
Ymodel.I{18} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{19}=0;
Ymodel.CoM{19}=[0; 0; 0];
Ymodel.I{19} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{20}=0;
Ymodel.CoM{20}=[0; 0; 0];
Ymodel.I{20} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{21}=0;
Ymodel.CoM{21}=[0; 0; 0];
Ymodel.I{21} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{22}=0;
Ymodel.CoM{22}=[0; 0; 0];
Ymodel.I{22} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{23}=0;
Ymodel.CoM{23}=[0; 0; 0];
Ymodel.I{23} = [0 0 0; 0 0 0; 0 0 0];

Ymodel.m{24}=0;
Ymodel.CoM{24}=[0; 0; 0];
Ymodel.I{24} = [0 0 0; 0 0 0; 0 0 0];
end
