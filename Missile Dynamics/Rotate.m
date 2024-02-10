function R = Rotate(ang,axis)
    if strcmp(axis,'x')
        R = R_x(ang);
    elseif strcmp(axis, 'y')
        R = R_y(ang);
    elseif strcmp(axis, 'z')
        R = R_z(ang);
    else
        R = eye(3);
    end
end

function Rx = R_x(Theta_x)
    Rx = [1 0 0;
          0 cos(Theta_x) sin(Theta_x);
          0 -sin(Theta_x) cos(Theta_x)];
end

function Ry = R_y(Theta_y)
    Ry = [cos(Theta_y) 0 -sin(Theta_y);
          0 1 0;
          sin(Theta_y) 0 cos(Theta_y)];
end

function Rz = R_z(Theta_z)
    Rz = [cos(Theta_z) sin(Theta_z) 0;
          -sin(Theta_z) cos(Theta_z) 0;
          0 0 1];
end
