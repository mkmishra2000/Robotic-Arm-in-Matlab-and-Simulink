% Vishnu... thank you for electronics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:- Manas Kumar Mishra
% Task:- find the joint space for a given tool space
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Output is the five angles [t1 t2 t3 t4 t5] 
% htm is homogeneous transformation matix.
% D1 is the base joint offset
% L2 is the link length (DH parameter of the second joint)
% L3 is the link length (DH parameter of the third joint)
% L4 is the link length (DH parameter of the forth joint)
% D5 is the end effect joint

function [t1,t2,t3,t4,t5] = rhino_inverse_kin(htm, D1, L2, L3, L4,D5)
%     x, y and z components
    x = htm(1,4);
    y = htm(2,4);
    z = htm(3,4);
    
    % Worksapce condition
    flag =1;
    max_length = L2+L3+L4+D5;

    if(sqrt(x^2 + y^2) > max_length)
        fprintf('This point is not possible at all...\n');
        flag = 0;
    end
    
%     initialize the angles
    t1 = 0;
    t2 = 0;
    t3 = 0;
    t4 = 0;
    t5 = 0;
    
    if(flag~=0)

        % thetha one calculation.
        z1 = htm(1,3);
        z2 = htm(2,3);

        t1 = rad2deg(atan2(z2, z1));
        

        % thetha five calculation
        x3 = htm(3,1);
        y3 = htm(3,2);

        t5 = rad2deg(atan2(-y3, x3));

        % sum of the angle thetha2, thetha3, thetha4
        t234 = rad2deg(atan2(-z1, cos(t1)));


        a = htm(1,4);
        c = htm(3,4);

        % L3*cos(t23)+L2*cos(t2) = C
        C = (a/cos(t1))-L4*cos(t234)+ D5*sin(t234);

        % L3*sin(t23) + L2*sin(t2) = D
        D =  D1 - c - D5*cos(t234) - L4*sin(t234);
        break_point = 0;

    %     inner workspace conditions
        if((((C^2)+(D^2)-(L3^2)-(L2^2))/(2*L3*L2)) >1 || (((C^2)+(D^2)-(L3^2)-(L2^2))/(2*L3*L2)< -1))
            break_point = 1;
        end

        if(break_point ==0)

            insito_cosine = ((C^2)+(D^2)-(L3^2)-(L2^2))/(2*L3*L2);
            insito_sine = sqrt(1-(insito_cosine)^2);

            t3 = rad2deg(atan2(insito_sine, insito_cosine));

            r = L3*insito_cosine + L2;
            s = L3*insito_sine;

            y_t2 = r*D-s*C;
            x_t2 = r*C+s*D;

            t2 = rad2deg(atan2(y_t2, x_t2));

            % t2+t3+t4 = t234
            t4 = t234-t2-t3;
            
        else
            fprintf('This configuration is not possible for joint 2, 3, 4...\n')
        end
    end
end