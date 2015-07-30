function  [HT_body, HT_ee] = Bioloid_T (q)
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
global srcLoaded Bioloidmodel
if isempty(Bioloidmodel)
    if isempty(srcLoaded)
        addpath(genpath('../../src'));
        display('--> Folder src and subfolders added to the path')
        srcLoaded = true;
    end
    Bioloid_Model();
    display('--> Bioloid model loaded')
end

HT_body = body_LF_Transform (Bioloidmodel.NB, Bioloidmodel.LFinPF, Bioloidmodel.axes, q, Bioloidmodel.parent);
HT_ee{1} = HT_body{9}*Bioloidmodel.EEinLF{1}; %End-effector Right Arm
HT_ee{2} = HT_body{12}*Bioloidmodel.EEinLF{2}; %End-effector Left Arm
HT_ee{3} = HT_body{18}*Bioloidmodel.EEinLF{3}; %End-effector Right Leg
HT_ee{4} = HT_body{24}*Bioloidmodel.EEinLF{4}; %End-effector Left Leg

end