% SCARA DH PARAMETER CALCULATIONS AND BASIC ANALYSIS.

%Vishnu... Thank you for electronics.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Robotics
%Manas Kumar Mishra (IIITDM kancheepuram)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%Iniviual transform matrix...
% it show how at the indiviual level scara is operating.

%    joint    theta     twist Angle    link length    base offset
%     j1      t1          0               0              199.2mm
%     j2      t2          0             400mm             59.5mm
%     j3       0        180             250mm             -d3
%     j4      t4          0               0               38.5mm

% htmb1 = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {sym('t1'),199.2,0,0})
% htmb2 = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {sym('t2'),59.5,400,0})
% htmb3 = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {0,-sym('d3'),250,pi})
% htmb4 = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {sym('t4'),38.5,0,0})


%%

%Forward Kinematic equation for SCARA.

% Scarafkin = htmb1*htmb2*htmb3*htmb4
% 
% Scarafkin = simplify(Scarafkin)

%%
%  symbolic representation of robotics

HTMSYM1 = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {sym('t1'),sym('L12'),sym('L11'),0})
HTMSYM2 = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {sym('t2'),0,sym('L2'),0})
HTMSYM3 = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {0,sym('d3'),0,pi})
HTMSYM4 = subs(HTM,{sym(thi),sym(dk),sym(ak),sym(alpha)}, {sym('t4'),sym('L4'),0,0})

%%
% basic all matrix for future while in jacobian...

T2from0 = simplify(HTMSYM1*HTMSYM2)

T3from0 = simplify(HTMSYM1*HTMSYM2*HTMSYM3)



%%

% Symbolic transformation matrix.

HTMSYMBOLIC = HTMSYM1*HTMSYM2*HTMSYM3*HTMSYM4

fprintf('Homogeneous transformation Matrix (forward kinematics)...')

HTMSYMBOLIC = simplify(HTMSYMBOLIC)


%%

% define the parameters of the scara 
% first base length d1 in cm...
L12 = 19.92;

% first link length L1 in cm...
L11 = 40;

% second link length L2 in cm...
L2 = 25;

% gripper position base in cm...
L4 = 3.85;


%%
% In this calculation I am not calculating the forth angle
% Because as of now my assumption is orientation of the 
% robot is the in indentity matrix.
% % Inverse kinematics analysis
% [t1, t2, d3, t4]
% x-coordinates...
x = 16;

% y -coordinates...
y = 15;

% z-coordinates...
z = 20;

% yaw angle in degree
yaw = 0;
y_r = deg2rad(yaw);

% pitch angle in degree
pitch = 0;
p_r = deg2rad(pitch);

% Roll angle in degree
roll =0;
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

R_mat = R_yaw*R_pitch*R_roll


fprintf('point of analysis...')
point = [x y z]'
htmat =[R_mat point; 0 0 0 1]
flag=0;
% check the boundry conditions...
if ((x^2 + y^2)>(L11+L2)^2)
    fprintf('This point is not possible...out of work space...\n');
    flag =1;
end

if(flag ~=1)
    d3 = z+L4-L12
    cost2 = (x^2+y^2-(L2^2)-(L11^2))/(2*L2*L11)
    t2 = acosd(cost2);
    t2 = deg2rad(t2);
    sint2 = sin(t2);
    t2 = atan2(sint2, cost2)*180/pi
     
    r = L2*cost2 + L11;
    s = L2*sint2;
    
    num = r*y-s*x;
    den = r*x+s*y;
    
    t1 = atan2d(num, den)
    cost4 = R_mat(1,1);
    t4 = t1+t2-acosd(cost4)
end

[T1, T2, D3, T4]= scara_invkin(htmat, L11, L12, L2, L4);

fprintf('Inverse kinematics...')
Dh_values =[t1, t2, d3, t4]'


%%
% jacobian analysis...
% using numerical method...
% first calculate the ROTATIONAL MATRIX for each step
R00 = [1 0 0; 0 1 0; 0 0 1];

R01 = HTMSYM1(1:3, 1:3);

R02 = T2from0(1:3, 1:3);

R03 = T3from0(1:3, 1:3);

% R04 Not necessarry just for idea about the rotational matrix
R04 = HTMSYMBOLIC(1:3, 1:3);

% Now position vector of each joint/motor/Actuator...

P04 = HTMSYMBOLIC(1:3, 4);

% position vectors of the prismatic joint is not necessary...
p03 =  T3from0(1:3,4);

p02 =  T2from0(1:3,4);

p01 =  HTMSYM1(1:3,4);

p00 = [0 0 0]';

i3 = [0 0 1];
% calculation of the b_ values eg; b1 = R01*i3

b0 = R00*(i3)';

b1 = R01*(i3)';

b2 = R02*(i3)';

b3 = R03*(i3)';

% now cross product of the b_ and position vector difference only for 
% ROTATIONAL JOINTS...

z1 = cross(b0, (P04-p00));

z2 = cross(b1, (P04-p01));

z3 = -b2;

z4 = cross(b3, (P04-p03));

% Last step jacobian calculation...

jacobianMat = [z1 z2 z3 z4; b0*1 b1*1 b2*0 b3*1]


%%
% substitute the value of robots into the jacobian matrix
% Wow jacobian depands only on the link lengths of the robots...
current_jacobianMat = subs(jacobianMat,{sym('L2'),sym('L11')}, {25,40})


%%

% set the initial and final points of the motion.
in_point =[50 30 19];

in_angle =[0 0 0];

Htm_in = HTMatrix4(in_point, in_angle);

fn_point =[20 20 16];

fn_angle =[0 0 0];

Htm_fn = HTMatrix4(fn_point, fn_angle);


%%

% inverse kinematics on the points

[ti1, ti2, di3, ti4] = scara_invkin(Htm_in, L11, L12, L2, L4);

[tf1, tf2, df3, tf4] = scara_invkin(Htm_fn, L11, L12, L2, L4);


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

A = subs(A,{sym('ti'),sym('tf')}, {ti, tf})


%%

% Initial accelaration and velocity...

Vi =0;
Vf =0;
ai =0;
af =0;


Bmat1 = [ti1;Vi;ai;tf1;Vf;af];
Bmat2 = [ti2;Vi;ai;tf2;Vf;af];
Bmat3 = [di3;Vi;ai;df3;Vf;af];
Bmat4 = [ti4;Vi;ai;tf4;Vf;af];

c1 = inv(A)*Bmat1;
c2 = inv(A)*Bmat2;
c3 = inv(A)*Bmat3;
c4 = inv(A)*Bmat4;


%%
% joint space method for trajectory
% for all values of t find angle,
% velocity,acceleration profiles.

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

for t=ti:0.1:tf-0.1
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
    
end
range = ti:0.1:tf-0.1;

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

toc

%%

% plot the point in 3-d space.
tic
X =[];
Y =[];
Z =[];
ja1 = deg2rad(jointAngle1);
ja2 = deg2rad(jointAngle2);
% ja4 = deg2rad(jointAngle4);
L = length(range);
% check =0
for i = 1:L
    xl(i) = L2*cos(ja1(i) + ja2(i)) + L11*cos(ja1(i));
    yl(i) = L2*sin(ja1(i) + ja2(i)) + L11*sin(ja1(i));
    zl(i) = L12 - L4 + jointAngle3(i);
    
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

% Scara structure (simulation)...


x_pos1 = L11*cosd(ti1);
y_pos1 = L11*sind(ti1);
x_pos2 = x_pos1+L2*cosd(ti2+ti1);
y_pos2 = y_pos1+L2*sind(ti2+ti1);
Z_3 = L12-L4+di3;

x_pos1f = L11*cosd(tf1);
y_pos1f = L11*sind(tf1);
x_pos2f = x_pos1f+ L2*cosd(tf2+tf1);
y_pos2f = y_pos1f+L2*sind(tf2+tf1);
Z_3f = L12-L4+df3;


figure;
hold on
plot3([0 0], [0 0], [0 L12],'k','linewidth',10)
plot3([0 x_pos1], [0 y_pos1], [L12 L12],'b','linewidth',10)
plot3([x_pos1 x_pos2], [y_pos1 y_pos2], [L12 L12],'r','linewidth',10)
plot3([x_pos2 x_pos2], [y_pos2 y_pos2], [L12 Z_3],'g','linewidth',10)
plot3(in_point(1), in_point(2), in_point(3),'o','linewidth',10)
plot3([0 0], [0 0], [0 L12],'k','linewidth',10)
plot3([0 x_pos1f], [0 y_pos1f], [L12 L12],'m','linewidth',10)
plot3([x_pos1f x_pos2f], [y_pos1f y_pos2f], [L12 L12],'m','linewidth',10)
plot3([x_pos2f x_pos2f], [y_pos2f y_pos2f], [L12 Z_3f],'m','linewidth',10)
plot3(fn_point(1), fn_point(2), fn_point(3),'o','linewidth',10)
xlabel('X-axis (cm)');
ylabel('Y-axis (cm)');
zlabel('Z-axis (cm)');
grid on;
axis square;
hold off;


%%

% Check the path and scara structure.

figure;
hold on
plot3([0 0], [0 0], [0 L12],'k','linewidth',10)
plot3([0 x_pos1], [0 y_pos1], [L12 L12],'b','linewidth',10)
plot3([x_pos1 x_pos2], [y_pos1 y_pos2], [L12 L12],'r','linewidth',10)
plot3([x_pos2 x_pos2], [y_pos2 y_pos2], [L12 Z_3],'g','linewidth',10)
plot3(in_point(1), in_point(2), in_point(3),'og','linewidth',10)
plot3(X,Y,Z,'<r','linewidth', 0.9);
plot3(X(1),Y(1),Z(1),'+','linewidth', 8)
plot3([0 0], [0 0], [0 L12],'k','linewidth',10)
plot3([0 x_pos1f], [0 y_pos1f], [L12 L12],'m','linewidth',10)
plot3([x_pos1f x_pos2f], [y_pos1f y_pos2f], [L12 L12],'m','linewidth',10)
plot3([x_pos2f x_pos2f], [y_pos2f y_pos2f], [L12 Z_3f],'m','linewidth',10)
plot3(fn_point(1), fn_point(2), fn_point(3),'o','linewidth',10)
xlabel('X-axis (cm)');
ylabel('Y-axis (cm)');
zlabel('Z-axis (cm)');
grid on;
% axis square;
hold off;


%%

% Lets simulate the path of the Scara robot...

tic
close all
fig = figure;
for j = 1:L
    clf(fig);
    x_pos1 = L11*cosd(jointAngle1(j));
    y_pos1 = L11*sind(jointAngle1(j));
    x_pos2 = x_pos1+L2*cosd(jointAngle2(j)+jointAngle1(j));
    y_pos2 = y_pos1+L2*sind(jointAngle2(j)+jointAngle1(j));
    Z_3 = L12-L4+jointAngle3(j);

    hold on
    plot3([0 x_pos1], [0 y_pos1], [L12 L12],'b','linewidth',15);
    plot3([x_pos1 x_pos2], [y_pos1 y_pos2], [L12 L12],'r','linewidth',15);
    plot3([x_pos2 x_pos2], [y_pos2 y_pos2], [L12 Z_3],'g','linewidth',10);
    plot3([0 0], [0 0], [0 L12],'k','linewidth',20);
    plot3(X,Y,Z,'<r','linewidth', 1.2);
    plot3(X(1),Y(1),Z(1),'+','linewidth', 8);
%     set(0, 'defaultFigureUnits', 'normalized')
%     plot3(in_point(1), in_point(2), in_point(3),'o','linewidth',10)
    grid on;
    xlim([-10, 90]);
    ylim([-10, 65]);
    xlabel('X-axis (cm)');
    ylabel('Y-axis (cm)');
    zlabel('Z-axis (cm)');
%     view([x y]);
    pause(0.0001);
    
end
plot3(X,Y,Z,'<r','linewidth', 1.2);
plot3(X(1),Y(1),Z(1),'+','linewidth', 8);
hold off;
toc

%%

% Simu parameters
t2 = double(ja1);

t3 = double(ja2);

jangle1 = timeseries(t2, range);

jangle2 = timeseries(t3, range);

%%
% THANK YOU


