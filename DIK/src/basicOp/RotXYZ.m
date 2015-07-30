function  R = RotXYZ (e)
R = [                                cos(e(3))*cos(e(2)),                                -cos(e(2))*sin(e(3)),            sin(e(2));
     cos(e(1))*sin(e(3)) + cos(e(3))*sin(e(1))*sin(e(2)), cos(e(1))*cos(e(3)) - sin(e(1))*sin(e(3))*sin(e(2)), -cos(e(2))*sin(e(1));
     sin(e(1))*sin(e(3)) - cos(e(1))*cos(e(3))*sin(e(2)), cos(e(3))*sin(e(1)) + cos(e(1))*sin(e(3))*sin(e(2)),  cos(e(1))*cos(e(2))];
end