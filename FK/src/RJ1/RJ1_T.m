function  [HT_body, HT_ee] = RJ1_T (q)
%% Homogeneous transformation matrices of each link of robot RJ1
% Inputs:
%    q = Robot configuration, i.e. each element of q is a joint position
% Outputs:
%    HT_body = Homogeneous transformation matrix wrt the world frame of the
%              beginning of each body after the variation of the joint which 
%              connect it with its predecessor.
%    HT_ee = Homogeneous transformation matrix of the end-effector wrt world 
%            frame
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global srcLoaded RJ1model
if isempty(RJ1model)
    if isempty(srcLoaded)
        addpath(genpath('../../src'));
        display('--> Folder src and subfolders added to the path')
        srcLoaded = true;
    end
    RJ1_Model();
    display('--> RJ1 model loaded')
end

HT_body = body_LF_Transform (RJ1model.NB, RJ1model.LFinPF, RJ1model.axes, q);
HT_ee{1} = HT_body{3}*RJ1model.EEinLF{9};
HT_ee{2}= HT_body{3}*RJ1model.EEinLF{12};
HT_ee{3}= HT_body{6}*RJ1model.EEinLF{18};
HT_ee{4}= HT_body{6}*RJ1model.EEinLF{24};
end