function R = Rot3(axes, angles)

R = Rot(axes(1), angles(1))*Rot(axes(1), angles(1))*Rot(axes(1), angles(1));

end