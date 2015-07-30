function Bioloid_Model()
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
global srcLoaded Bioloidmodel
if isempty(srcLoaded)
    addpath(genpath('../../src'));
    display('--> Folder src and subfolders added to the path')
    srcLoaded = true;
end

Bioloidmodel.NB = 24;
Bioloidmodel.link=    [ 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 ];%1to6 referential frame to Body frame, 7to9 Right Arm, 10to12 Left Arm, 13to18 Right Leg, 19to24 Left Leg
Bioloidmodel.axes =   [ 4 5 6 1 2 3 2 1 1 2  1  1  3  1  2  2  2  1  3  1  2  2  2  1 ];
Bioloidmodel.parent = [ 0 1 2 3 4 5 6 7 8 6  10 11 6  13 14 15 16 17 6  19 20 21 22 23 ];

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
Bioloidmodel.jntVelLim = [ inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf];
Bioloidmodel.jntLim = [ inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf;inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf];

%% Position of the referential frame of each body in the parent coordinates
%"Variable" distances between joints

%Model "Real" Distances (SolidWorks)
l1=0.0145;%Distance in X axis
l2=0.0385;%Distances in Y axis
l3=0.049;
l4=0.023;
l5=0.0675;
l6=0.0855;
l7=0.0145;%Distance in Z axis
l8=0.1205;
l9=0.075;
l10=0.075;
l11=0.032;

Bioloidmodel.LFinPF{1} = Tras([0,0,0]); %6 DOF of the "base"
Bioloidmodel.LFinPF{2} = Tras([0,0,0]);
Bioloidmodel.LFinPF{3} = Tras([0,0,0]);
Bioloidmodel.LFinPF{4} = Tras([0,0,0]);
Bioloidmodel.LFinPF{5} = Tras([0,0,0]);
Bioloidmodel.LFinPF{6} = Tras([0,0,0]);

Bioloidmodel.LFinPF{7} = Tras([0,-l3,0]); %3 DOF of the Right arm
Bioloidmodel.LFinPF{8} = Tras([0,-l4,-l7]);
Bioloidmodel.LFinPF{9} = Tras([0,-l5,0]);

Bioloidmodel.LFinPF{10} = Tras([0,l3,0]); %3 DOF of the Left arm
Bioloidmodel.LFinPF{11} = Tras([0,l4,-l7]);
Bioloidmodel.LFinPF{12} = Tras([0,l5,0]);

Bioloidmodel.LFinPF{13} = Tras([0,-l2,-l8]); %6 DOF of the Right leg %With the real measurements l7 was deleted from here
Bioloidmodel.LFinPF{14} = Tras([0,0,0]);
Bioloidmodel.LFinPF{15} = Tras([0,0,0]);
Bioloidmodel.LFinPF{16} = Tras([-l1,0,-l9]);
Bioloidmodel.LFinPF{17} = Tras([l1,0,-l10]);
Bioloidmodel.LFinPF{18} = Tras([0,0,0]);

Bioloidmodel.LFinPF{19} = Tras([0,l2,-l8]); %6 DOF of the Left leg %With the real measurements l7 was deleted from here
Bioloidmodel.LFinPF{20} = Tras([0,0,0]);
Bioloidmodel.LFinPF{21} = Tras([0,0,0]);
Bioloidmodel.LFinPF{22} = Tras([-l1,0,-l9]);
Bioloidmodel.LFinPF{23} = Tras([l1,0,-l10]);
Bioloidmodel.LFinPF{24} = Tras([0,0,0]);

%% Position of the referential frame of the end-effector in the body local frame
Bioloidmodel.EEinLF{1} = Tras([0,-l6,0]);%End-effector Right hand
Bioloidmodel.EEinLF{2} = Tras([0,l6,0]);%End-effector Left hand
Bioloidmodel.EEinLF{3} = Tras([0,0,-l11]);%End-effector Right leg
Bioloidmodel.EEinLF{4} = Tras([0,0,-l11]);%End-effector Left leg

%% Position of the wrist wrt the end-efector frame
%Bioloidmodel.eePw = -Bioloidmodel.EEinLF{1}(1:3,1:3).'*(Bioloidmodel.EEinLF{1}(1:3,4) + [0; 0; l]);
%It hasn't been designed due to we are not using it, it has no changes from
%the original.
%% dynamic parameters
Bioloidmodel.m{1}=0;
Bioloidmodel.CoM{1}=[0; 0; 0];
Bioloidmodel.I{1} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{2}=0;
Bioloidmodel.CoM{2}=[0; 0; 0];
Bioloidmodel.I{2} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{3}=0;
Bioloidmodel.CoM{3}=[0; 0; 0];
Bioloidmodel.I{3} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{4}=0;
Bioloidmodel.CoM{4}=[0; 0; 0];
Bioloidmodel.I{4} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{5}=0;
Bioloidmodel.CoM{5}=[0; 0; 0];
Bioloidmodel.I{5} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{6}=0;
Bioloidmodel.CoM{6}=[0; 0; 0];
Bioloidmodel.I{6} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{7}=0;
Bioloidmodel.CoM{7}=[0; 0; 0];
Bioloidmodel.I{7} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{8}=0;
Bioloidmodel.CoM{8}=[0; 0; 0];
Bioloidmodel.I{8} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{9}=0;
Bioloidmodel.CoM{9}=[0; 0; 0];
Bioloidmodel.I{9} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{10}=0;
Bioloidmodel.CoM{10}=[0; 0; 0];
Bioloidmodel.I{10} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{11}=0;
Bioloidmodel.CoM{11}=[0; 0; 0];
Bioloidmodel.I{11} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{12}=0;
Bioloidmodel.CoM{12}=[0; 0; 0];
Bioloidmodel.I{12} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{13}=0;
Bioloidmodel.CoM{13}=[0; 0; 0];
Bioloidmodel.I{13} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{14}=0;
Bioloidmodel.CoM{14}=[0; 0; 0];
Bioloidmodel.I{14} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{15}=0;
Bioloidmodel.CoM{15}=[0; 0; 0];
Bioloidmodel.I{15} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{16}=0;
Bioloidmodel.CoM{16}=[0; 0; 0];
Bioloidmodel.I{16} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{17}=0;
Bioloidmodel.CoM{17}=[0; 0; 0];
Bioloidmodel.I{17} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{18}=0;
Bioloidmodel.CoM{18}=[0; 0; 0];
Bioloidmodel.I{18} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{19}=0;
Bioloidmodel.CoM{19}=[0; 0; 0];
Bioloidmodel.I{19} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{20}=0;
Bioloidmodel.CoM{20}=[0; 0; 0];
Bioloidmodel.I{20} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{21}=0;
Bioloidmodel.CoM{21}=[0; 0; 0];
Bioloidmodel.I{21} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{22}=0;
Bioloidmodel.CoM{22}=[0; 0; 0];
Bioloidmodel.I{22} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{23}=0;
Bioloidmodel.CoM{23}=[0; 0; 0];
Bioloidmodel.I{23} = [0 0 0; 0 0 0; 0 0 0];

Bioloidmodel.m{24}=0;
Bioloidmodel.CoM{24}=[0; 0; 0];
Bioloidmodel.I{24} = [0 0 0; 0 0 0; 0 0 0];
end
