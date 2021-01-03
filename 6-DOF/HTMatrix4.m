% Vishnu... Thank you for electronics
% Author:- Manas Kumar Mishra

% Find Homogeneous transformation matrix from the position 
% vector and angles between the fixed frame and
% mobile frame (in degree).

% PV is position vector [x y z]
% yaw angle about z-axis
% pitch angle about y-axis
% roll angle about x-axis.

function Htmatrix = HTMatrix4(PV, E_angle)
    
    yaw = E_angle(1);
    pitch = E_angle(2);
    roll = E_angle(3);
    
    y_r = deg2rad(yaw);
    p_r = deg2rad(pitch);
    r_r =deg2rad(roll);
    
    R_yaw = [cos(y_r) -sin(y_r) 0; 
         sin(y_r)  cos(y_r) 0;
         0 0 1];

    R_pitch = [cos(p_r)  0 sin(p_r); 
             0 1 0;
           -sin(p_r) 0 cos(p_r)];
       
    R_roll = [1 0 0;
              0 cos(r_r) -sin(r_r); 
              0 sin(r_r)  cos(r_r);];

    R_mat = R_yaw*R_pitch*R_roll;
    scale =1;
    
    Htmatrix = [R_mat PV'; 0 0 0 scale];
end

