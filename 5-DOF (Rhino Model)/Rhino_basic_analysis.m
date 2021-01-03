% Vishnu...Thank you for electronics

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:- MANAS KUMAR MISHRA
% Task:- Design the 5 DOF Robotics (RHINO).
% Organization:- IIITDM KANCHEEPURAM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% RIHNO DH PARAMETER CALCULATION AND BASIC ANALYSIS
%%
%1.DH parameter matrix.
tic
close all
clear all
% DH parameters...
syms thi dk ak alpha

%Joint angle (joint variable)
%Rotation of frame about z-axis.
Rtheta = [cos(thi) -sin(thi) 0 0;
          sin(thi)  cos(thi) 0 0;
          0 0 1 0; 0 0 0 1]

%Joint offset or joint distance (Joint variable).
%Translation along z-axis.
Tdk = [1 0 0 0;
       0 1 0 0;
       0 0 1 dk;
       0 0 0 1]
   
%Link length (Link parameter)
%Translation along common normal(x-axis)
Tak = [1 0 0 ak;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1]

%Twist angle (Link parameter) 
%Rotation about common normal(x-axis)
Ralpha =[1 0 0 0;
         0 cos(alpha) -sin(alpha) 0;
         0 sin(alpha)  cos(alpha) 0;
         0 0 0 1]
toc


%%

%Overall DH (single frame matrix)

fprintf('DH parameter Matrix...\n')

HTM =Tdk*Rtheta*Tak*Ralpha

%%

% jointAngle   jointOffset   linklength   twistAngle
%     t1          260           0           -pi/2
%     t2          0           228            0
%     t3          0           228            0
%     t4          0            90           -pi/2
%     t5          171.5         0            0
%     

HTMSYM1_rhino = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {sym('t1'),sym('D1'),0,sym('Tw1')})
HTMSYM2_rhino = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {sym('t2'),0,sym('L2'),0})
HTMSYM3_rhino = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {sym('t3'),0,sym('L3'),0})
HTMSYM4_rhino = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {sym('t4'),0,sym('L4'),sym('Tw2')})
HTMSYM5_rhino = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {sym('t5'),sym('D5'),0,0})

%%

% Symbolic transformation matrix.

HTMSYMBOLIC_rhino = HTMSYM1_rhino*HTMSYM2_rhino*HTMSYM3_rhino*HTMSYM4_rhino*HTMSYM5_rhino

fprintf('Homogeneous transformation Matrix (forward kinematics)...')

HTMSYMBOLIC_rhino = simplify(HTMSYMBOLIC_rhino)


%%

% Twist angle 1
Tw1 = -pi/2;

% Twist angle 2
Tw2 = -pi/2;

T1from0 = simplify(subs(HTMSYM1_rhino,{sym('Tw1'),sym('Tw2')}, {Tw1,Tw2}))


T2from0 = simplify(HTMSYM1_rhino*HTMSYM2_rhino)
T2from0 = simplify(subs(T2from0,{sym('Tw1'),sym('Tw2')}, {Tw1,Tw2}))

T3from0 = simplify(HTMSYM1_rhino*HTMSYM2_rhino*HTMSYM3_rhino)
T3from0 = simplify(subs(T3from0,{sym('Tw1'),sym('Tw2')}, {Tw1,Tw2}))

T4from0 = simplify(HTMSYM1_rhino*HTMSYM2_rhino*HTMSYM3_rhino*HTMSYM4_rhino)
T4from0 = simplify(subs(T4from0,{sym('Tw1'),sym('Tw2')}, {Tw1,Tw2}))

T5from0 = simplify(HTMSYM1_rhino*HTMSYM2_rhino*HTMSYM3_rhino*HTMSYM4_rhino*HTMSYM5_rhino)
T5from0 = simplify(subs(T5from0,{sym('Tw1'),sym('Tw2')}, {Tw1,Tw2}))


%%

% Homogeneous transformation matrix only for joint angles.

% Joint offset one base side
D1 = 26;

% Joint offset on the link/end-effector side
D5 = 17.15;

% Twist angle 1
Tw1 = -pi/2;

% Twist angle 2
Tw2 = -pi/2;

% link length
L2 =22.8;

% Link length 3
L3 = 22.8;

% Link Length 4
L4 = 9.0;

%%

% Now I want to check the Homogenous matrix by putting the values of Twist
% Angle 1 and 2 into the previous matrix.

fprintf('After putting the twist angle...\n ')
check_twist = simplify(subs(HTMSYMBOLIC_rhino,{sym('Tw1'),sym('Tw2')}, {Tw1,Tw2}))

Check_HTMatrix = simplify(subs(HTMSYMBOLIC_rhino,{sym('D1'),sym('D5'),sym('Tw1'),sym('Tw2')}, {D1,D5,Tw1,Tw2}))

% Now I want to check the final HTM by putting the link lengths
Check_by_link = subs(Check_HTMatrix,{sym('L2'),sym('L3'),sym('L4')}, {L2,L3,L4})

%%
% Default position and orientation analysis
% find the HTM 

x = 31.8;
y = 10;
z = 38.65;

pos = [x y z];
angle = [0 0 0];

Def_mat = HTMatrix4(pos, angle)

%%

% Now inverse Kinematics...
tic

% Worksapce condition
flag =1;
max_length = L2+L3+L4+D5;

if(sqrt(x^2 + y^2) > max_length)
    fprintf('This point is not possible at all...\n');
    flag = 0;
end

if(flag~=0)
    
    % thetha one calculation.
    z1 = Def_mat(1,3);
    z2 = Def_mat(2,3);

    t1 = atan2(z2, z1)
    T1 = rad2deg(t1)

    % thetha five calculation
    x3 = Def_mat(3,1);
    y3 = Def_mat(3,2);

    t5 = atan2(-y3, x3)
    T5 = rad2deg(t5)

    % sum of the angle thetha2, thetha3, thetha4
    t234 = atan2(-z1, cos(t1))
    T234 = rad2deg(t234)


    a = Def_mat(1,4)
    c = Def_mat(3,4)

    % L3*cos(t23)+L2*cos(t2) = C
    C = (a/cos(t1))-L4*cos(t234)+ D5*sin(t234)

    % L3*sin(t23) + L2*sin(t2) = D
    D =  D1 - c - D5*cos(t234) - L4*sin(t234)
    t3 = 0;
    t2 = 0;
    t4 = 0;
    T3 = 0;
    T4 = 0;
    T2 = 0;
    
    break_point = 0;
    
%     inner workspace conditions
    if((((C^2)+(D^2)-(L3^2)-(L2^2))/(2*L3*L2)) >1 || ((C^2)+(D^2)-(L3^2)-(L2^2))/(2*L3*L2)< -1)
        break_point = 1;
    end
    
    if(break_point ==0)
        
        insito_cosine = ((C^2)+(D^2)-(L3^2)-(L2^2))/(2*L3*L2);
        insito_sine = sqrt(1-(insito_cosine)^2);

        t3 = atan2(insito_sine, insito_cosine)
        T3 = rad2deg(t3)

        r = L3*insito_cosine + L2;
        s = L3*insito_sine;

        y_t2 = r*D-s*C;
        x_t2 = r*C+s*D;

        t2 = atan2(y_t2, x_t2)
        T2 = rad2deg(t2)

        % t2+t3+t4 = t234
        t4 = t234-t2-t3
        T4 = rad2deg(t4)
    else
        fprintf('This configuration is not possible for joint 2, 3, 4...\n')
    end
%     Accumlation of the angles
    fprintf('Angles in radian...\n');
    t = [t1 t2 t3 t4 t5]
    
    fprintf('Angles in Degree...\n');
    T = [T1 T2 T3 T4 T5]

end
toc

%%

% Now jacobian matrix calculation
% since DOF is five... n=5
% matrix dimension is 6*5

% Rotation matrix from base to base
R00 = [1 0 0;0 1 0; 0 0 1];

% Rotation matrix for base to first joint
R01 = T1from0(1:3, 1:3);

% Rotation matrix for base to second joint
R02 = T2from0(1:3, 1:3);

% Rotation matrix for base to third joint
R03 = T3from0(1:3, 1:3);

% Rotation matrix for base to third joint
R04 = T4from0(1:3, 1:3);

% Rotation matrix for base to third joint
R05 = T5from0(1:3, 1:3);

% position vector from 0 to base.
p00 = [0 0 0]';

% position vector from 0 to first.
p01 = T1from0(1:3,4);

% position vector from 0 to second.
p02 = T2from0(1:3,4);

% position vector from 0 to third.
p03 = T3from0(1:3,4);

% position vector from 0 to forth.
p04 = T4from0(1:3,4);

% position vector from 0 to fifth.
p05 = T5from0(1:3,4);

% for last column 
i3 = [0 0 1];

% calculation of the b_ values eg; b1 = R01*i3

b0 = R00*i3';

b1 = R01*i3';

b2 = R02*i3';

b3 = R03*i3';

b4 = R04*i3';

b5 = R05*i3';

% now cross product of the b_ and position vector difference only for 
% ROTATIONAL JOINTS...

z1 = simplify(cross(b0, (p05-p00)));

z2 = simplify(cross(b1, (p05-p01)));

z3 = simplify(cross(b2, (p05-p02)));

z4 = simplify(cross(b3, (p05-p03)));

z5 = simplify(cross(b4, (p05-p04)));

% last step to find jacobian matrix...
jacobian = [z1 z2 z3 z4 z5; b0*1 b1*1 b2*1 b3*1 b4*1]


%%

jacobian = simplify(subs(jacobian,{sym('D1'),sym('L2'),sym('L3'),sym('L4'),sym('D5')}, {D1,L2,L3,L4,D5}))

 

%%

% set the initial and final points of the motion.
in_point =[10 20 20];

in_angle =[30 0 0];

Htm_in = HTMatrix4(in_point, in_angle)

fn_point =[50 10 10];

fn_angle =[10 0 0];

Htm_fn = HTMatrix4(fn_point, fn_angle)

%%

% initial angle set
[ti1 ti2 ti3 ti4 ti5] = rhino_inverse_kin(Htm_in, D1, L2, L3, L4, D5)

% final angle set
[tf1 tf2 tf3 tf4 tf5] = rhino_inverse_kin(Htm_fn, D1, L2, L3, L4, D5)


%%

% set the Higher order polynomial scheme...
syms ti tf
A = [1 ti (ti)^2 (ti)^3 (ti)^4 (ti)^5;
     0 1 2*(ti) 3*(ti)^2 4*(ti)^3 5*(ti)^4;
     0 0 2 6*(ti) 12*(ti)^2 20*(ti)^3;
     1 tf (tf)^2 (tf)^3 (tf)^4 (tf)^5;
     0 1 2*(tf) 3*(tf)^2 4*(tf)^3 5*(tf)^4;
     0 0 2 6*(tf) 12*(tf)^2 20*(tf)^3];

 % Time in sec
ti =0;
tf =5;

% time matrix
A = subs(A,{sym('ti'),sym('tf')}, {ti, tf})

%%

% Initial accelaration and velocity...

Vi =0;
Vf =0;
ai =0;
af =0;


Bmat1 = [ti1;Vi;ai;tf1;Vf;af];
Bmat2 = [ti2;Vi;ai;tf2;Vf;af];
Bmat3 = [ti3;Vi;ai;tf3;Vf;af];
Bmat4 = [ti4;Vi;ai;tf4;Vf;af];
Bmat5 = [ti5;Vi;ai;tf5;Vf;af];

c1 = inv(A)*Bmat1;
c2 = inv(A)*Bmat2;
c3 = inv(A)*Bmat3;
c4 = inv(A)*Bmat4;
c5 = inv(A)*Bmat5;

%%

% joint space method for trajectory
% for all values of t find angle,
% velocity,acceleration profiles.
fprintf("Wait...\n")
tic

jointAngle1 =[];
jointVelocity1 = [];
jointAccelration1 = [];

jointAngle2 =[];
jointVelocity2 = [];
jointAccelration2 = [];

jointAngle3 =[];
jointVelocity3 = [];
jointAccelration3 = [];

jointAngle4 =[];
jointVelocity4 = [];
jointAccelration4 = [];

jointAngle5 =[];
jointVelocity5 = [];
jointAccelration5 = [];

for t=ti:0.05:tf-0.05
    Q1 = c1(1,1)+c1(2,1)*t+c1(3,1)*t^2+c1(4,1)*t^3+c1(5,1)*t^4+c1(6,1)*t^5;
    V1 = c1(2,1)+2*c1(3,1)*t+3*c1(4,1)*t^2+4*c1(5,1)*t^3+5*c1(6,1)*t^4;
    a1 = 2*c1(3,1)+6*c1(4,1)*t+12*c1(5,1)*t^2+20*c1(6,1)*t^3;
    jointAngle1 = [jointAngle1,Q1];
    jointVelocity1 = [jointVelocity1, V1];
    jointAccelration1 = [jointAccelration1, a1];
    
    Q2 = c2(1,1)+c2(2,1)*t+c2(3,1)*t^2+c2(4,1)*t^3+c2(5,1)*t^4+c2(6,1)*t^5;
    V2 = c2(2,1)+2*c2(3,1)*t+3*c2(4,1)*t^2+4*c2(5,1)*t^3+5*c2(6,1)*t^4;
    a2 = 2*c2(3,1)+6*c2(4,1)*t+12*c2(5,1)*t^2+20*c2(6,1)*t^3;
    jointAngle2 = [jointAngle2,Q2];
    jointVelocity2 = [jointVelocity2, V2];
    jointAccelration2 = [jointAccelration2, a2];
    
    Q3 = c3(1,1)+c3(2,1)*t+c3(3,1)*t^2+c3(4,1)*t^3+c3(5,1)*t^4+c3(6,1)*t^5;
    V3 = c3(2,1)+2*c3(3,1)*t+3*c3(4,1)*t^2+4*c3(5,1)*t^3+5*c3(6,1)*t^4;
    a3 = 2*c3(3,1)+6*c3(4,1)*t+12*c3(5,1)*t^2+20*c3(6,1)*t^3;
    jointAngle3 = [jointAngle3,Q3];
    jointVelocity3 = [jointVelocity3, V3];
    jointAccelration3 = [jointAccelration3, a3];
    
    Q4 = c4(1,1)+c4(2,1)*t+c4(3,1)*t^2+c4(4,1)*t^3+c4(5,1)*t^4+c4(6,1)*t^5;
    V4 = c4(2,1)+2*c4(3,1)*t+3*c4(4,1)*t^2+4*c4(5,1)*t^3+5*c4(6,1)*t^4;
    a4 = 2*c4(3,1)+6*c4(4,1)*t+12*c4(5,1)*t^2+20*c4(6,1)*t^3;
    jointAngle4 = [jointAngle4,Q4];
    jointVelocity4 = [jointVelocity4, V4];
    jointAccelration4 = [jointAccelration4, a4];
    
    Q5 = c5(1,1)+c5(2,1)*t+c5(3,1)*t^2+c5(4,1)*t^3+c5(5,1)*t^4+c5(6,1)*t^5;
    V5 = c5(2,1)+2*c5(3,1)*t+3*c5(4,1)*t^2+4*c5(5,1)*t^3+5*c5(6,1)*t^4;
    a5 = 2*c5(3,1)+6*c5(4,1)*t+12*c5(5,1)*t^2+20*c5(6,1)*t^3;
    jointAngle5 = [jointAngle5,Q5];
    jointVelocity5 = [jointVelocity5, V5];
    jointAccelration5 = [jointAccelration5, a5];
    
end

range = ti:0.05:tf-0.05;

figure;
subplot(2,2,[1,2]);
plot(range, jointAngle1,'*', 'linewidth', 0.5);
xlabel('time');
ylabel('Angle');
grid on;
title('Joint angle-1');

subplot(2,2,3);
plot(range, jointVelocity1,'r', 'linewidth', 1);
xlabel('time');
ylabel('Velocity');
grid on;
title('first Joint velocity ');

subplot(2,2,4);
plot(range, jointAccelration1,'m', 'linewidth', 1);
xlabel('time');
ylabel('Accelration');
grid on;
title('first Joint Accelration');



figure;
subplot(2,2,[1,2]);
plot(range, jointAngle2,'*', 'linewidth', 0.5);
xlabel('time');
ylabel('Angle');
grid on;
title('Joint angle-2');

subplot(2,2,3);
plot(range, jointVelocity2,'r', 'linewidth', 1);
xlabel('time');
ylabel('Velocity');
grid on;
title('second Joint velocity ');

subplot(2,2,4);
plot(range, jointAccelration2,'m', 'linewidth', 1);
xlabel('time');
ylabel('Accelration');
grid on;
title('second Joint Accelration');


figure;
subplot(2,2,[1,2]);
plot(range, jointAngle3,'*', 'linewidth', 0.5);
xlabel('time');
ylabel('Angle');
grid on;
title('Joint angle-3');

subplot(2,2,3);
plot(range, jointVelocity3,'r', 'linewidth', 1);
xlabel('time');
ylabel('Velocity');
grid on;
title('third Joint velocity ');

subplot(2,2,4);
plot(range, jointAccelration3,'m', 'linewidth', 1);
xlabel('time');
ylabel('Accelration');
grid on;
title('third Joint Accelration');



figure;
subplot(2,2,[1,2]);
plot(range, jointAngle4,'*', 'linewidth', 0.5);
xlabel('time');
ylabel('Angle');
grid on;
title('Joint angle-4');

subplot(2,2,3);
plot(range, jointVelocity4,'r', 'linewidth', 1);
xlabel('time');
ylabel('Velocity');
grid on;
title('forth Joint velocity ');

subplot(2,2,4);
plot(range, jointAccelration4,'m', 'linewidth', 1);
xlabel('time');
ylabel('Accelration');
grid on;
title('forth Joint Accelration');


figure;
subplot(2,2,[1,2]);
plot(range, jointAngle5,'*', 'linewidth', 0.5);
xlabel('time');
ylabel('Angle');
grid on;
title('Joint angle-5');

subplot(2,2,3);
plot(range, jointVelocity5,'r', 'linewidth', 1);
xlabel('time');
ylabel('Velocity');
grid on;
title('fifth Joint velocity ');

subplot(2,2,4);
plot(range, jointAccelration5,'m', 'linewidth', 1);
xlabel('time');
ylabel('Accelration');
grid on;
title('fifth Joint Accelration');


toc

%%

fprintf('WAIT...\n')
% plot the point in 3-d space.
tic
X =[];
Y =[];
Z =[];
ja1 = deg2rad(jointAngle1);
ja2 = deg2rad(jointAngle2);
ja3 = deg2rad(jointAngle3);
ja4 = deg2rad(jointAngle4);
ja5 = deg2rad(jointAngle5);

L = length(range);
% check =0
for i = 1:L
    xl(i) = cos(ja1(i))*( L3*cos(ja2(i)+ja3(i)) + L2*(cos(ja2(i))) + L4*(cos(ja2(i)+ja3(i)+ja4(i))) - D5*(sin(ja2(i)+ja3(i)+ja4(i))));
    yl(i) = sin(ja1(i))*( L3*cos(ja2(i)+ja3(i)) + L2*(cos(ja2(i))) + L4*(cos(ja2(i)+ja3(i)+ja4(i))) - D5*(sin(ja2(i)+ja3(i)+ja4(i))));
    zl(i) = D1-L3*sin(ja2(i)+ja3(i))-L2*sin(ja2(i))-D5*cos(ja2(i)+ja3(i)+ja4(i))-L4*sin(ja2(i)+ja3(i)+ja4(i));
    
    X = [X, xl];
    Y = [Y, yl];
    Z = [Z, zl];
end

figure;
plot3(X,Y,Z,'^','linewidth', 0.6);
hold on
plot3(X(1),Y(1),Z(1),'+','linewidth', 8)
xlabel('X-axis (cm)');
ylabel('Y-axis (cm)');
zlabel('Z-axis (cm)');
grid on;
axis square;
hold off;

toc

%%

% Rhino structure simulation...
tic
x_pos1 = L2*cos(ja2(1))*cos(ja1(1));
y_pos1 = L2*cos(ja2(1))*sin(ja1(1));
z_pos1 = -L2*sin(ja2(1))+D1;

x_pos2 = (L3*cos(ja3(1)+ja2(1))+L2*cos(ja2(1)))*cos(ja1(1));
y_pos2 = (L3*cos(ja3(1)+ja2(1))+L2*cos(ja2(1)))*sin(ja1(1));
z_pos2 = -L3*sin(ja3(1)+ja2(1))-L2*sin(ja2(1))+D1;

x_pos3 = (L3*cos(ja3(1)+ja2(1))+L2*cos(ja2(1))+L4*cos(ja2(1)+ja3(1)+ja4(1)))*cos(ja1(1));
y_pos3 = (L3*cos(ja3(1)+ja2(1))+L2*cos(ja2(1))+L4*cos(ja2(1)+ja3(1)+ja4(1)))*sin(ja1(1));
z_pos3 = -L3*sin(ja3(1)+ja2(1))-L2*sin(ja2(1))+D1-L4*sin(ja2(1)+ja3(1)+ja4(1));

x_pos4 = (L3*cos(ja3(1)+ja2(1))+L2*cos(ja2(1))+L4*cos(ja2(1)+ja3(1)+ja4(1)) - D5*sin(ja2(1)+ja3(1)+ja4(1)))*cos(ja1(1));
y_pos4 = (L3*cos(ja3(1)+ja2(1))+L2*cos(ja2(1))+L4*cos(ja2(1)+ja3(1)+ja4(1)) - D5*sin(ja2(1)+ja3(1)+ja4(1)))*sin(ja1(1));
z_pos4 =  -L3*sin(ja3(1)+ja2(1))-L2*sin(ja2(1))+D1-L4*sin(ja2(1)+ja3(1)+ja4(1))-D5*cos(ja2(1)+ja3(1)+ja4(1));

x_pos1f = L2*cos(ja2(L))*cos(ja1(L));
y_pos1f = L2*cos(ja2(L))*sin(ja1(L));
z_pos1f = -L2*sin(ja2(L))+D1;

x_pos2f = (L3*cos(ja3(L)+ja2(L))+L2*cos(ja2(L)))*cos(ja1(L));
y_pos2f = (L3*cos(ja3(L)+ja2(L))+L2*cos(ja2(L)))*sin(ja1(L));
z_pos2f = -L3*sin(ja3(L)+ja2(L))-L2*sin(ja2(L))+D1;

x_pos3f = (L3*cos(ja3(L)+ja2(L))+L2*cos(ja2(L))+L4*cos(ja2(L)+ja3(L)+ja4(L)))*cos(ja1(L));
y_pos3f = (L3*cos(ja3(L)+ja2(L))+L2*cos(ja2(L))+L4*cos(ja2(L)+ja3(L)+ja4(L)))*sin(ja1(L));
z_pos3f = -L3*sin(ja3(L)+ja2(L))-L2*sin(ja2(L))+D1-L4*sin(ja2(L)+ja3(L)+ja4(L));

x_pos4f = (L3*cos(ja3(L)+ja2(L))+L2*cos(ja2(L))+L4*cos(ja2(L)+ja3(L)+ja4(L)) - D5*sin(ja2(L)+ja3(L)+ja4(L)))*cos(ja1(L));
y_pos4f = (L3*cos(ja3(L)+ja2(L))+L2*cos(ja2(L))+L4*cos(ja2(L)+ja3(L)+ja4(L)) - D5*sin(ja2(L)+ja3(L)+ja4(L)))*sin(ja1(L));
z_pos4f =  -L3*sin(ja3(L)+ja2(L))-L2*sin(ja2(L))+D1-L4*sin(ja2(L)+ja3(L)+ja4(L))-D5*cos(ja2(L)+ja3(L)+ja4(L));

figure;
hold on;
plot3([0 0], [0 0], [0 D1], 'k','linewidth', 50)
plot3([0 x_pos1],[0 y_pos1],[D1 z_pos1],'b','linewidth',10)
plot3([x_pos1 x_pos2],[y_pos1 y_pos2],[z_pos1 z_pos2],'r','linewidth',10)
plot3([x_pos2 x_pos3],[y_pos2 y_pos3],[z_pos2 z_pos3],'g','linewidth',10)
plot3([x_pos3 x_pos4],[y_pos3 y_pos4],[z_pos3 z_pos4],'m','linewidth',10)
plot3(X,Y,Z,'^','linewidth', 0.6);
plot3(X(1),Y(1),Z(1),'+','linewidth', 8)
plot3([0 x_pos1f],[0 y_pos1f],[D1 z_pos1f],'y','linewidth',10)
plot3([x_pos1f x_pos2f],[y_pos1f y_pos2f],[z_pos1f z_pos2f],'c','linewidth',10)
plot3([x_pos2f x_pos3f],[y_pos2f y_pos3f],[z_pos2f z_pos3f],'y','linewidth',10)
plot3([x_pos3f x_pos4f],[y_pos3f y_pos4f],[z_pos3f z_pos4f],'c','linewidth',10)
grid on;
xlabel('X-axis (cm)');
ylabel('Y-axis (cm)');
zlabel('Z-axis (cm)');
hold off;

toc


%%

% Lets simulate the path of the Rhino robot...



tic
close all
fig = figure;
for j = 1:L
    clf(fig);
    x_pos1 = L2*cos(ja2(j))*cos(ja1(j));
    y_pos1 = L2*cos(ja2(j))*sin(ja1(j));
    z_pos1 = -L2*sin(ja2(j))+D1;

    x_pos2 = (L3*cos(ja3(j)+ja2(j))+L2*cos(ja2(j)))*cos(ja1(j));
    y_pos2 = (L3*cos(ja3(j)+ja2(j))+L2*cos(ja2(j)))*sin(ja1(j));
    z_pos2 = -L3*sin(ja3(j)+ja2(j))-L2*sin(ja2(j))+D1;

    x_pos3 = (L3*cos(ja3(j)+ja2(j))+L2*cos(ja2(j))+L4*cos(ja2(j)+ja3(j)+ja4(j)))*cos(ja1(j));
    y_pos3 = (L3*cos(ja3(j)+ja2(j))+L2*cos(ja2(j))+L4*cos(ja2(j)+ja3(j)+ja4(j)))*sin(ja1(j));
    z_pos3 = -L3*sin(ja3(j)+ja2(j))-L2*sin(ja2(j))+D1-L4*sin(ja2(j)+ja3(j)+ja4(j));

    x_pos4 = (L3*cos(ja3(j)+ja2(j))+L2*cos(ja2(j))+L4*cos(ja2(j)+ja3(j)+ja4(j)) - D5*sin(ja2(j)+ja3(j)+ja4(j)))*cos(ja1(j));
    y_pos4 = (L3*cos(ja3(j)+ja2(j))+L2*cos(ja2(j))+L4*cos(ja2(j)+ja3(j)+ja4(j)) - D5*sin(ja2(j)+ja3(j)+ja4(j)))*sin(ja1(j));
    z_pos4 =  -L3*sin(ja3(j)+ja2(j))-L2*sin(ja2(j))+D1-L4*sin(ja2(j)+ja3(j)+ja4(j))-D5*cos(ja2(j)+ja3(j)+ja4(j));


    hold on
    plot3([0 0], [0 0], [0 D1], 'k','linewidth', 50)
    plot3([0 x_pos1],[0 y_pos1],[D1 z_pos1],'b','linewidth',10)
    plot3([x_pos1 x_pos2],[y_pos1 y_pos2],[z_pos1 z_pos2],'r','linewidth',10)
    plot3([x_pos2 x_pos3],[y_pos2 y_pos3],[z_pos2 z_pos3],'g','linewidth',10)
    plot3([x_pos3 x_pos4],[y_pos3 y_pos4],[z_pos3 z_pos4],'m','linewidth',10)
    grid on;
    xlim([-50, 90]);
    ylim([-50, 65]);
    zlim([0, 70]);
    xlabel('X-axis (cm)');
    ylabel('Y-axis (cm)');
    zlabel('Z-axis (cm)');
    view([x y]);
    pause(0.0001);
    
end
plot3(X,Y,Z,'<r','linewidth', 1.2);
plot3(X(1),Y(1),Z(1),'+','linewidth', 8);
hold off;
toc

%%
% simu interface
tic

% change the symbol format into double
simu_ang1 = double(ja1);
simu_ang2 = double(ja2);
simu_ang3 = double(ja3);
simu_ang4 = double(ja4);
simu_ang5 = double(ja5);

acc1 = timeseries(simu_ang1, range);
acc2 = timeseries(simu_ang2, range);
acc3 = timeseries(simu_ang3, range);
acc4 = timeseries(simu_ang4, range);
acc5 = timeseries(simu_ang5, range);
toc
%%
% Thank you...
