function  out = ToSimulink_Y_FK_ST (q)
%% Forward kinemtics by means Successive Transformations
% Inputs: 
%    q = Robot configuration, i.e. each element of q is a joint position
%
% Outputs: 
%    T_ee(1:3,1:3) = End-effector rotation matrix 
%    T_ee(1:3,4) = End-effector position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global srcLoaded Ymodel
if isempty(Ymodel)
    if isempty(srcLoaded)
        addpath(genpath('src'));
        display('--> Folder src and subfolders added to the path')
        srcLoaded = true;
    end
    Y_Model();
    display('--> Y model loaded')
end

[~, T_ee] = Y_T (q);

E_RH=EulerXYZ(T_ee{1}(1:3,1:3));
E_LH=EulerXYZ(T_ee{2}(1:3,1:3));

out = [T_ee{1}(1:3,4);E_RH.';T_ee{2}(1:3,4);E_LH.'];

end
