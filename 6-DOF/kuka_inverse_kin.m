% Vishnu... thank you for electronics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:- Manas Kumar Mishra
% Task:- find the joint space for a given tool space
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Output is the five angles [t1 t2 t3 t4 t5 t6] 
% htm is homogeneous transformation matix.
% D1 is the base joint offset
% L2 is the link length (DH parameter of the second joint)
% L3 is the link length (DH parameter of the third joint)


function [t1,t2,t3,t4,t5,t6]= kuka_inverse_kin(htm,D1,L2,L3)
    % x component
    pw_x= htm(1,4); 

    % y component
    pw_y=htm(2,4);

    % z component
    pw_z=htm(3,4);

    % rotation matrix
    R = htm(1:3,1:3);

    % note that I need to ensure that c3 value lies in [-1, 1]
    c3=(pw_x^2+pw_y^2+pw_z^2-L2^2-L3^2)/(2*L2*L3);
    t1=0; t2=0; t3=0; t4=0; t5=0; t6=0;
    
    if(c3<=1 && c3>=-1)

        s3=-sqrt(1-c3^2);
        t3=rad2deg(atan2((s3),(c3)));
        

        t2=rad2deg(atan2(((L2+L3*c3)*pw_z-L3*s3*sqrt(pw_x^2+pw_y^2)),((L2+L3*c3)*sqrt(pw_x^2+pw_y^2)+L3*s3*pw_z)));

        t1=rad2deg(atan2(pw_y,pw_x));
        

        R3_0=[cos(t1)*cos(t2+t3) -cos(t1)*sin(t2+t3) sin(t1); 
        sin(t1)*cos(t2+t3) -sin(t1)*sin(t2+t3) -cos(t1);
        sin(t2+t3) cos(t2+t3) 0];
        R6_3=R3_0*R; % Matrix for the Euler's angle of spherical wrist
        % Inverse kinematic for the spherical wrist

        t4= rad2deg(atan2(R6_3(2,3),R6_3(1,3)));
        

        t5=rad2deg(atan2(sqrt((R6_3(1,3))^2+(R6_3(2,3))^2),R6_3(3,3)));


        t6=rad2deg(atan2(R6_3(3,2),R6_3(3,1)));
       
    else
        fprintf("Not in the workspace !!!\n");
    end
end
