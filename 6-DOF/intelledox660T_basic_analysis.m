
% Vishnu...Thank you for electronics

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:- MANAS KUMAR MISHRA
% Task:- Design the 6 DOF Robotics.
% Organization:- IIITDM KANCHEEPURAM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% DH PARAMETER CALCULATION AND BASIC ANALYSIS
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
%     t1          10            0            pi/2
%     t2          0            50            pi
%     t3          0            30           -pi/2
%     t4          0             0            pi/2
%     t5          0             0            pi/2
%     t6          0             0            pi/2


HTMSYM1_intelldox = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {sym('t1'),sym('D1'),0,sym('Tw1')})
HTMSYM2_intelldox = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {sym('t2'),0,sym('L2'),sym('Tw2')})
HTMSYM3_intelldox = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {sym('t3'),0,sym('L3'),sym('Tw3')})
HTMSYM4_intelldox = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {sym('t4'),0,0,sym('Tw4')})
HTMSYM5_intelldox = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {sym('t5'),0,0,sym('Tw5')})
HTMSYM6_intelldox = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {sym('t6'),0,0,sym('Tw6')})

%%
% Symbolic transformation matrix.

HTMSYMBOLIC_intelldox = HTMSYM1_intelldox*HTMSYM2_intelldox*HTMSYM3_intelldox*HTMSYM4_intelldox*HTMSYM5_intelldox*HTMSYM6_intelldox

fprintf('Homogeneous transformation Matrix (forward kinematics)...\n')

HTMSYMBOLIC_intelldox = simplify(HTMSYMBOLIC_intelldox)


%%
tic
% Twist angle 1
Tw1 = pi/2;

% Twist angle 2
Tw2 = pi;

% Twist angle 3
Tw3 = -pi/2;

% Twist angle 4
Tw4 = pi/2;

% Twist angle 5
Tw5 = -pi/2;

% Twist angle 6
Tw6 = pi/2;

T1from0 = simplify(subs(HTMSYM1_intelldox,{sym('Tw1'),sym('Tw2'),sym('Tw3'),sym('Tw4'),sym('Tw5'),sym('Tw6')}, {Tw1,Tw2,Tw3,Tw4,Tw5,Tw6}))


T2from0 = simplify(HTMSYM1_intelldox*HTMSYM2_intelldox);
T2from0 = simplify(subs(T2from0,{sym('Tw1'),sym('Tw2'),sym('Tw3'),sym('Tw4'),sym('Tw5'),sym('Tw6')}, {Tw1,Tw2,Tw3,Tw4,Tw5,Tw6}))

T3from0 = simplify(HTMSYM1_intelldox*HTMSYM2_intelldox*HTMSYM3_intelldox);
T3from0 = simplify(subs(T3from0,{sym('Tw1'),sym('Tw2'),sym('Tw3'),sym('Tw4'),sym('Tw5'),sym('Tw6')}, {Tw1,Tw2,Tw3,Tw4,Tw5,Tw6}))

T4from0 = simplify(HTMSYM1_intelldox*HTMSYM2_intelldox*HTMSYM3_intelldox*HTMSYM4_intelldox);
T4from0 = simplify(subs(T4from0,{sym('Tw1'),sym('Tw2'),sym('Tw3'),sym('Tw4'),sym('Tw5'),sym('Tw6')}, {Tw1,Tw2,Tw3,Tw4,Tw5,Tw6}))

T5from0 = simplify(HTMSYM1_intelldox*HTMSYM2_intelldox*HTMSYM3_intelldox*HTMSYM4_intelldox*HTMSYM5_intelldox);
T5from0 = simplify(subs(T5from0,{sym('Tw1'),sym('Tw2'),sym('Tw3'),sym('Tw4'),sym('Tw5'),sym('Tw6')}, {Tw1,Tw2,Tw3,Tw4,Tw5,Tw6}))

forward_matrix = simplify(subs(HTMSYMBOLIC_intelldox,{sym('Tw1'),sym('Tw2'),sym('Tw3'),sym('Tw4'),sym('Tw5'),sym('Tw6')}, {Tw1,Tw2,Tw3,Tw4,Tw5,Tw6}))
toc

%%

% Homogeneous transformation matrix only for joint angles.

% Joint offset one base side
D1 = 10;

% Joint offset on the link/end-effector side
% D6 = 17.15;

% Twist angle 1
Tw1 = -pi/2;

% Twist angle 4
Tw4 = -pi/2;

% Twist angle 5
Tw5 = -pi/2;

% Twist angle 6
Tw6 = -pi/2;

% link length
L2 =50;

% Link length 3
L3 = 30;

% Link Length 4
% L4 = 9.0;

%%


% Now I want to check the Homogenous matrix by putting the values of Twist
% Angle 1 and 2 into the previous matrix.

fprintf('After putting the twist angle...\n ')
% check_twist = simplify(subs(HTMSYMBOLIC_intelldox,{sym('Tw1'),sym('Tw4'),sym('Tw5'),sym('Tw6')}, {Tw1,Tw4,Tw5,Tw6}))

Check_HTMatrix = simplify(subs(forward_matrix,{sym('D1')}, {D1}))

% Now I want to check the final HTM by putting the link lengths
Check_by_link = simplify(subs(Check_HTMatrix,{sym('L2'),sym('L3')}, {L2,L3}))



%%
% Default position and orientation analysis
% find the HTM 

x = 59.8;
y = 10;
z = 38.65;

pos = [x y z];
angle = [0 0 0];

Def_mat = HTMatrix4(pos, angle)

%%

% inverse kinematics for intelledox 6 DOF

% nx ox ax px
% ny oy ay py
% nz oz az pz
% 0  0  0  1

tic
% x component
pw_x=Def_mat(1,4); 

% y component
pw_y=Def_mat(2,4);

% z component
pw_z=Def_mat(3,4);

% rotation matrix
R = Def_mat(1:3,1:3);

% note that I need to ensure that c3 value lies in [-1, 1]
c3=(pw_x^2+pw_y^2+pw_z^2-L2^2-L3^2)/(2*L2*L3)
t1=0; t2=0; t3=0; t4=0; t5=0; t6=0;
T1=0;T2=0;T3=0;T4=0;T5=0;T6=0;
if(c3<=1 && c3>=-1)

    s3=-sqrt(1-c3^2)
    t3=atan2((s3),(c3))
    T3 = rad2deg(t3)

    t2=atan2(((L2+L3*c3)*pw_z-L3*s3*sqrt(pw_x^2+pw_y^2)),((L2+L3*c3)*sqrt(pw_x^2+pw_y^2)+L3*s3*pw_z))
    T2 = rad2deg(t2)

    t1=atan2(pw_y,pw_x)
    T1=rad2deg(t1)

    R3_0=[cos(t1)*cos(t2+t3) -cos(t1)*sin(t2+t3) sin(t1); 
    sin(t1)*cos(t2+t3) -sin(t1)*sin(t2+t3) -cos(t1);
    sin(t2+t3) cos(t2+t3) 0];
    R6_3=R3_0*R; % Matrix for the Euler's angle of spherical wrist
    % Inverse kinematic for the spherical wrist

    t4=atan2(R6_3(2,3),R6_3(1,3))
    T4 = rad2deg(t4)

    t5=atan2(sqrt((R6_3(1,3))^2+(R6_3(2,3))^2),R6_3(3,3))
    T5 = rad2deg(t5)


    t6=atan2(R6_3(3,2),R6_3(3,1))
    T6 = rad2deg(t6)
else
    fprintf("Not in the workspace !!!\n");
end

fprintf('Angle in radian...\n');
t=[t1 t2 t3 t4 t5 t6]

fprintf('Angle in degree...\n');
T=[T1 T2 T3 T4 T5 T6]
toc

%%

% set the initial and final points of the motion.
in_point =[50 40 40];

in_angle =[30 0 0];

Htm_in = HTMatrix4(in_point, in_angle)

fn_point =[20 10 10];

fn_angle =[0 0 10];

Htm_fn = HTMatrix4(fn_point, fn_angle)



% initial angle set
[ti1 ti2 ti3 ti4 ti5 ti6] = kuka_inverse_kin(Htm_in, D1, L2, L3)

% final angle set
[tf1 tf2 tf3 tf4 tf5 tf6] = kuka_inverse_kin(Htm_fn, D1, L2, L3)


%%
% Let's start from here.
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
Bmat6 = [ti6;Vi;ai;tf6;Vf;af];

c1 = inv(A)*Bmat1;
c2 = inv(A)*Bmat2;
c3 = inv(A)*Bmat3;
c4 = inv(A)*Bmat4;
c5 = inv(A)*Bmat5;
c6 = inv(A)*Bmat6;

%%

% joint space method for trajectory
% for all values of t find angle,
% velocity,acceleration profiles.
fprintf("Wait...calculating the angle...\n");
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

jointAngle6 =[];
jointVelocity6 = [];
jointAccelration6 = [];

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
    
    Q6 = c6(1,1)+c6(2,1)*t+c6(3,1)*t^2+c6(4,1)*t^3+c6(5,1)*t^4+c6(6,1)*t^5;
    V6 = c6(2,1)+2*c6(3,1)*t+3*c6(4,1)*t^2+4*c6(5,1)*t^3+5*c6(6,1)*t^4;
    a6 = 2*c6(3,1)+6*c6(4,1)*t+12*c6(5,1)*t^2+20*c6(6,1)*t^3;
    jointAngle6 = [jointAngle6,Q6];
    jointVelocity6 = [jointVelocity6, V6];
    jointAccelration6 = [jointAccelration6, a6];
    
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

figure;
subplot(2,2,[1,2]);
plot(range, jointAngle6,'*', 'linewidth', 0.5);
xlabel('time');
ylabel('Angle');
grid on;
title('Joint angle-6');

subplot(2,2,3);
plot(range, jointVelocity6,'r', 'linewidth', 1);
xlabel('time');
ylabel('Velocity');
grid on;
title('sixth Joint velocity ');

subplot(2,2,4);
plot(range, jointAccelration6,'m', 'linewidth', 1);
xlabel('time');
ylabel('Accelration');
grid on;
title('sixth Joint Accelration');

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
ja6 = deg2rad(jointAngle6);

L = length(range);
% check =0
for i = 1:L
    xl(i) = cos(ja1(i))*(L3*cos(ja2(i) - ja3(i)) + L2*cos(ja2(i)));
    yl(i) = sin(ja1(i))*(L3*cos(ja2(i) - ja3(i)) + L2*cos(ja2(i)));
    zl(i) = L3*sin(ja2(i) - ja3(i)) + L2*sin(ja2(i)) + D1;
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

% intelledox structure simulation...
fprintf('Wait...\n');
tic
x_pos1 = L2*cos(ja1(1))*cos(ja2(1));
y_pos1 = L2*cos(ja2(1))*sin(ja1(1));
z_pos1 = L2*sin(ja2(1))+D1;

x_pos2 = cos(ja1(1))*(L2*cos(ja2(1)) + L3*cos(ja2(1) - ja3(1)));
y_pos2 = sin(ja1(1))*(L2*cos(ja2(1)) + L3*cos(ja2(1) - ja3(1)));
z_pos2 = L2*sin(ja2(1))+D1+L3*sin(ja2(1)-ja3(1));

x_pos3 = cos(ja1(1))*(L2*cos(ja2(1)) + L3*cos(ja2(1) - ja3(1)));
y_pos3 = sin(ja1(1))*(L2*cos(ja2(1)) + L3*cos(ja2(1) - ja3(1)));
z_pos3 = L2*sin(ja2(1))+D1+L3*sin(ja2(1)-ja3(1));

x_pos4 = cos(ja1(1))*(L2*cos(ja2(1)) + L3*cos(ja2(1) - ja3(1)));
y_pos4 = sin(ja1(1))*(L2*cos(ja2(1)) + L3*cos(ja2(1) - ja3(1)));
z_pos4 = L2*sin(ja2(1))+D1+L3*sin(ja2(1)-ja3(1));


x_pos5 = cos(ja1(1))*(L2*cos(ja2(1)) + L3*cos(ja2(1) - ja3(1)));
y_pos5 = sin(ja1(1))*(L2*cos(ja2(1)) + L3*cos(ja2(1) - ja3(1)));
z_pos5 = L2*sin(ja2(1))+D1+L3*sin(ja2(1)-ja3(1));


x_pos1f = L2*cos(ja1(L))*cos(ja2(L));
y_pos1f = L2*cos(ja2(L))*sin(ja1(L));
z_pos1f = L2*sin(ja2(L))+D1;

x_pos2f = cos(ja1(L))*(L2*cos(ja2(L)) + L3*cos(ja2(L) - ja3(L)));
y_pos2f = sin(ja1(L))*(L2*cos(ja2(L)) + L3*cos(ja2(L) - ja3(L)));
z_pos2f = L2*sin(ja2(L))+D1+L3*sin(ja2(L)-ja3(L));

x_pos3f = cos(ja1(L))*(L2*cos(ja2(L)) + L3*cos(ja2(L) - ja3(L)));
y_pos3f = sin(ja1(L))*(L2*cos(ja2(L)) + L3*cos(ja2(L) - ja3(L)));
z_pos3f = L2*sin(ja2(L))+D1+L3*sin(ja2(L)-ja3(L));

x_pos4f = cos(ja1(L))*(L2*cos(ja2(L)) + L3*cos(ja2(L) - ja3(L)));
y_pos4f = sin(ja1(L))*(L2*cos(ja2(L)) + L3*cos(ja2(L) - ja3(L)));
z_pos4f = L2*sin(ja2(L))+D1+L3*sin(ja2(L)-ja3(L));


x_pos5f = cos(ja1(L))*(L2*cos(ja2(L)) + L3*cos(ja2(L) - ja3(L)));
y_pos5f = sin(ja1(L))*(L2*cos(ja2(L)) + L3*cos(ja2(L) - ja3(L)));
z_pos5f = L2*sin(ja2(L))+D1+L3*sin(ja2(L)-ja3(L));


figure;
hold on;
plot3([0 0], [0 0], [0 D1], 'k','linewidth', 50)
plot3([0 x_pos1],[0 y_pos1],[D1 z_pos1],'b','linewidth',10)
plot3([x_pos1 x_pos2],[y_pos1 y_pos2],[z_pos1 z_pos2],'r','linewidth',10)
plot3([x_pos2 x_pos3],[y_pos2 y_pos3],[z_pos2 z_pos3],'g','linewidth',10)
plot3([x_pos3 x_pos4],[y_pos3 y_pos4],[z_pos3 z_pos4],'m','linewidth',10)
plot3([x_pos4 x_pos5],[y_pos4 y_pos5],[z_pos4 z_pos5],'m','linewidth',10)
plot3(X,Y,Z,'^','linewidth', 0.6);
plot3(X(1),Y(1),Z(1),'+','linewidth', 8)
plot3([0 x_pos1f],[0 y_pos1f],[D1 z_pos1f],'y','linewidth',10)
plot3([x_pos1f x_pos2f],[y_pos1f y_pos2f],[z_pos1f z_pos2f],'c','linewidth',10)
plot3([x_pos2f x_pos3f],[y_pos2f y_pos3f],[z_pos2f z_pos3f],'y','linewidth',10)
plot3([x_pos3f x_pos4f],[y_pos3f y_pos4f],[z_pos3f z_pos4f],'c','linewidth',10)
plot3([x_pos4f x_pos5f],[y_pos4f y_pos5f],[z_pos4f z_pos5f],'Y','linewidth',10)

grid on;
xlabel('X-axis (cm)');
ylabel('Y-axis (cm)');
zlabel('Z-axis (cm)');
hold off;

toc

%%

% Lets simulate the path of the  robot...

tic
close all
fig = figure;
for j = 1:L
    clf(fig);
    x_pos1 = L2*cos(ja1(j))*cos(ja2(j));
    y_pos1 = L2*cos(ja2(j))*sin(ja1(j));
    z_pos1 = L2*sin(ja2(j))+D1;

    x_pos2 = cos(ja1(j))*(L2*cos(ja2(j)) + L3*cos(ja2(j) - ja3(j)));
    y_pos2 = sin(ja1(j))*(L2*cos(ja2(j)) + L3*cos(ja2(j) - ja3(j)));
    z_pos2 = L2*sin(ja2(j))+D1+L3*sin(ja2(j)-ja3(j));

    x_pos3 = cos(ja1(j))*(L2*cos(ja2(j)) + L3*cos(ja2(j) - ja3(j)));
    y_pos3 = sin(ja1(j))*(L2*cos(ja2(j)) + L3*cos(ja2(j) - ja3(j)));
    z_pos3 = L2*sin(ja2(j))+D1+L3*sin(ja2(j)-ja3(j));

    x_pos4 = cos(ja1(j))*(L2*cos(ja2(j)) + L3*cos(ja2(j) - ja3(j)));
    y_pos4 = sin(ja1(j))*(L2*cos(ja2(j)) + L3*cos(ja2(j) - ja3(j)));
    z_pos4 = L2*sin(ja2(j))+D1+L3*sin(ja2(j)-ja3(j));


    x_pos5 = cos(ja1(j))*(L2*cos(ja2(j)) + L3*cos(ja2(j) - ja3(j)));
    y_pos5 = sin(ja1(j))*(L2*cos(ja2(j)) + L3*cos(ja2(j) - ja3(j)));
    z_pos5 = L2*sin(ja2(j))+D1+L3*sin(ja2(j)-ja3(j));
    
    hold on
    plot3([0 0], [0 0], [0 D1], 'k','linewidth', 50)
    plot3([0 x_pos1],[0 y_pos1],[D1 z_pos1],'b','linewidth',10)
    plot3([x_pos1 x_pos2],[y_pos1 y_pos2],[z_pos1 z_pos2],'g','linewidth',10)
    plot3([x_pos2 x_pos3],[y_pos2 y_pos3],[z_pos2 z_pos3],'g','linewidth',10)
    plot3([x_pos3 x_pos4],[y_pos3 y_pos4],[z_pos3 z_pos4],'m','linewidth',10)
    plot3([x_pos4 x_pos5],[y_pos4 y_pos5],[z_pos4 z_pos5],'m','linewidth',10)
    grid on;
    xlim([-50, 90]);
    ylim([-50, 65]);
    zlim([0, 60])
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
simu_ang6 = double(ja6);


acc1 = timeseries(simu_ang1, range);
acc2 = timeseries(simu_ang2, range);
acc3 = timeseries(simu_ang3, range);
acc4 = timeseries(simu_ang4, range);
acc5 = timeseries(simu_ang5, range);
acc6 = timeseries(simu_ang6, range);

toc

%
%%

% Thank you
