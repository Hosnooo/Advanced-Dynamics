function [Coef_Fxyz, Coef_Momxyz] = Aerodynamic_Model(u, v, w, p, q, r,...
    Mach_number, alpha)
    load("Model_Params.mat")
    % Angle of attack & Side Slip
    [alpha, beta] = angles(u, v, w);
    
    if (sign(alpha) == 1 && Mach_number < 3) || (sign(alpha) == -1 && Mach_number >=3)
        Cm_alpha_val = -0.14;
    else
        Cm_alpha_val = 0.14;
    end

    Cm_el_val = Cm_el;
    Cm_q_val = Cm_q;
    
    Cx_alpha_val = -0.3;
    if sign(alpha) == 1
        Cz_alpha_val = -0.25;
    else
        Cz_alpha_val = 0.25;
    end
    
    Cz_el_val = Cz_el;

    % Aerodynamic Coefficients
    Cx = Cx_alpha_val*alpha;
  
    Cy_beta = -0.4;
    Cy = Cy_beta * beta;
    
    Cz = Cz_alpha_val*alpha;

    Cm = Cm_q_val*q + Cm_alpha_val*alpha;

    Cl_beta = -0.01;
    Cl = Cl_beta * beta;
    
    Cn_beta = 0.3;
    Cn = Cn_beta * beta;

    Cn_r = -0.2;
    Cn = Cn + Cn_r * r;
  
    Coef_Fxyz = [Cx; Cy; Cz];

    % Body Moments Coefficients
    Coef_Momxyz = [Cl; Cm; Cn];
end

function [angle_of_attack, side_slip] = angles(u, v, w)
    angle_of_attack = atan2(w, u);
    side_slip = asin(v / norm([u, v, w]));
end

function R = Rotate(angle, axis)
    if axis == 'y'
        R = [cos(angle), 0, sin(angle); 0, 1, 0; -sin(angle), 0, cos(angle)];
    elseif axis == 'z'
        R = [cos(angle), -sin(angle), 0; sin(angle), cos(angle), 0; 0, 0, 1];
    else
        error('Invalid axis. Use ''y'' or ''z''.');
    end
end
