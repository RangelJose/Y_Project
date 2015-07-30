function  out = Y_Jg_SV (q)
%% Geometric Jacobian matrix
% Inputs:
%    q = Robot configuration, i.e. each element of q is a joint position
% Outputs:
%    Jg = Geometric Jacobian matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global srcLoaded Ymodel 
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


T=body_LF_Transform (Ymodel.NB, Ymodel.LFinPF, Ymodel.axes, q, Ymodel.parent);
T_ee{1} = T{5}*Ymodel.EEinLF{1}; %End-effector Right Arm
T_ee{2} = T{6}*Ymodel.EEinLF{2}; %End-effector Left Arm

O_n{1} = T_ee{1}(1:3,4);
O_n{2} = T_ee{2}(1:3,4);

Jg{1} = Jg_EE(Ymodel.NB,O_n{1},T,5,Ymodel.parent,Ymodel.axes);
Jg{2} = Jg_EE(Ymodel.NB,O_n{2},T,6,Ymodel.parent,Ymodel.axes);


%% Outputs
out = [Jg{1}(:,1);Jg{1}(:,2);Jg{1}(:,3);Jg{1}(:,4);Jg{1}(:,5);Jg{1}(:,6);O_n{1};
       Jg{2}(:,1);Jg{2}(:,2);Jg{2}(:,3);Jg{2}(:,4);Jg{2}(:,5);Jg{2}(:,6);O_n{2}];