function e = EulerXZX(R)
% R = [         cos(e2),                          -cos(e3)*sin(e2),                             sin(e2)*sin(e3);
%       cos(e1)*sin(e2), cos(e1)*cos(e2)*cos(e3) - sin(e1)*sin(e3), - cos(e3)*sin(e1) - cos(e1)*cos(e2)*sin(e3);
%       sin(e1)*sin(e2), cos(e1)*sin(e3) + cos(e2)*cos(e3)*sin(e1),   cos(e1)*cos(e3) - cos(e2)*sin(e1)*sin(e3)];

e(1) = atan2(R(3,1),R(2,1));
e(2) = atan2(sqrt(R(1,3)^2 + R(1,2)^2),R(1,1));
e(3) = atan2(R(1,3),-R(1,2));
end