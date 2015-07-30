function  [HT_body, HT_ee] = Y_T (q)
%% Homogeneous transformation matrices of each link of robot Bioloid
% Inputs:
%    q = Robot configuration, i.e. each element of q is a joint position
% Outputs:
%    HT_body = Homogeneous transformation matrix wrt the world frame of the
%              beginning of each body after the variation of the joint which 
%              connect it with its predecessor.
%    HT_ee = Homogeneous transformation matrix of the end-effector wrt world 
%            frame
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global srcLoaded Ymodel
if isempty(Ymodel)
    if isempty(srcLoaded)
        addpath(genpath('../../src'));
        display('--> Folder src and subfolders added to the path')
        srcLoaded = true;
    end
    Y_Model();
    display('--> Y model loaded')
end

HT_body = body_LF_Transform (Ymodel.NB, Ymodel.LFinPF, Ymodel.axes, q, Ymodel.parent);
HT_ee{1} = HT_body{5}*Ymodel.EEinLF{1}; %End-effector Right Arm
HT_ee{2} = HT_body{6}*Ymodel.EEinLF{2}; %End-effector Left Arm


end