function RJ1_Model()
global srcLoaded RJ1model
if isempty(srcLoaded)
    addpath(genpath('../../src'));
    display('--> Folder src and subfolders added to the path')
    srcLoaded = true;
end

RJ1model.NB = 6;                 
%model.link=      [ 1 2 3 4 5 6 ];
RJ1model.axes =   [ 3 2 2 1 3 1 ];
RJ1model.parent = [ 0 1 2 3 4 5 ];

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
RJ1model.jntVelLim = [ inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf];
RJ1model.jntLim = [ inf -inf; inf -inf; inf -inf; inf -inf; inf -inf; inf -inf];

%% Position of the referential frame of each body in the parent coordinates              
RJ1model.LFinPF{1} = Tras([-1.5, 3, 0])*RotZ(deg2rad(25))*Tras([0, 0, 0.5]);
RJ1model.LFinPF{2} = Tras([0, 0, 0.6]);
RJ1model.LFinPF{3} = RotY(-deg2rad(30))*Tras([0, 0, 2]);
RJ1model.LFinPF{4} = RotY(deg2rad(30))*Tras([1.9, 0, 0]);
RJ1model.LFinPF{5} = Tras([0.29, 0, 0]);
RJ1model.LFinPF{6} = Tras([0.14, 0, 0]);

%% Position of the referential frame of the end-effector in the body local frame
RJ1model.EEinLF = Tras([0.05, 0, 0])*RotY(-deg2rad(50))*Tras([0.2, 0, 0])*RotY(deg2rad(60))*Tras([0.35, 0, 0]);

%% Position of the wrist wrt the end-efector frame
RJ1model.eePw = -RJ1model.EEinLF(1:3,1:3).'*(RJ1model.EEinLF(1:3,4) + [0.14; 0; 0]);

%% dynamic parameters
RJ1model.m{1}=0;
RJ1model.CoM{1}=[0; 0; 0];
RJ1model.I{1} = [0 0 0; 0 0 0; 0 0 0];

RJ1model.m{2}=0;
RJ1model.CoM{2}=[0; 0; 0];
RJ1model.I{2} = [0 0 0; 0 0 0; 0 0 0];

RJ1model.m{3}=0;
RJ1model.CoM{3}=[0; 0; 0];
RJ1model.I{3} = [0 0 0; 0 0 0; 0 0 0];

RJ1model.m{4}=0;
RJ1model.CoM{4}=[0; 0; 0];
RJ1model.I{4} = [0 0 0; 0 0 0; 0 0 0];

RJ1model.m{5}=0;
RJ1model.CoM{5}=[0; 0; 0];
RJ1model.I{5} = [0 0 0; 0 0 0; 0 0 0];

RJ1model.m{6}=0;
RJ1model.CoM{6}=[0; 0; 0];
RJ1model.I{6} = [0 0 0; 0 0 0; 0 0 0];
end
