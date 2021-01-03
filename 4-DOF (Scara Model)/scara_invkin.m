% Vishnu... thank you for electronic
% Author:- Manas Kumar Mishra.

% Inverse kinematic function for scara robot.
% htm is homogenous transformation matrix (4*4).
% L11 is first link length
% L12 is base length
% L2 is second link length
% L4 end-effect side link length.

function [t1, t2, d3, t4]= scara_invkin(htm, L11, L12, L2, L4)
%     x , y and z components
    x = htm(1,4);
    y = htm(2,4);
    z = htm(3,4);
    
    t1 =0;
    t2 =0;
    d3 =0;
    t4 =0;
    
    flag=0;
% check the boundry conditions...
    if ((x^2 + y^2)>(L11+L2)^2)
        fprintf('This point is not possible...out of work space...\n');
        flag =1;
    end
    if(flag~=1)
    %    prismatic joint variable
        d3 = z+L4-L12;

    %    joint-2 angle
        cost2 = (x^2+y^2-(L2^2)-(L11^2))/(2*L2*L11);
        t2 = acosd(cost2);
        t2 = deg2rad(t2);
        sint2 = sin(t2);
        t2 = atan2(sint2, cost2)*180/pi;

    %     joint-1 angle
        r = L2*cost2 + L11;
        s = L2*sint2;

        num = r*y-s*x;
        den = r*x+s*y;

        t1 = atan2d(num, den);

    %     joint-4 angle
        cost4 = htm(1,1);
        t4 = t1+t2-acosd(cost4);
    end
    
end