function T = body_LF_Transform (nb, bodyLFinPF, qAxes, q, parent)
%Each element in T{i} is the transformation matrix from the inertial frame to
%the beginning of body i after the variation of q(i)
T = cell(nb,1);
T{1} = bodyLFinPF{1}*HT(qAxes(1), q(1));

for i = 2:nb
    T{i} = T{parent(i)}*bodyLFinPF{i}*HT(qAxes(i), q(i));
    %It has been changed to calculate the kinematics chain based on the
    %parent of each joint
end
end