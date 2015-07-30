function e = EulerXYZ(R)
% R = [                          cos(e3)*cos(e2),                          -cos(e2)*sin(e3),          sin(e2);
%      cos(e1)*sin(e3) + cos(e3)*sin(e1)*sin(e2), cos(e1)*cos(e3) - sin(e1)*sin(e3)*sin(e2), -cos(e2)*sin(e1);
%      sin(e1)*sin(e3) - cos(e1)*cos(e3)*sin(e2), cos(e3)*sin(e1) + cos(e1)*sin(e3)*sin(e2),  cos(e1)*cos(e2)];

e(1) = atan2(-R(2,3),R(3,3));
e(2) = atan2(R(1,3),sqrt(R(2,3)^2 + R(3,3)^2));
e(3) = atan2(-R(1,2),R(1,1));
end