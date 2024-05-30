clc
clear

x=-90; % Roll 0
y=0; % Pitch 90
z=0; % Yaw 0
RR = compose_rotation(x, y, z)
%

L1 = 0.729; L2 = 1.546; L3 = 1.949; L4 = 0.084; L5 = 1.505; L6 = 0.968;

% thetha1 = 30; thetha2 = 15; thetha3 = 120; thetha4 = 5; thetha5 = -60; thetha6 = 10;
% 4.085, 1.38, 2.32
T = [RR(1,1)    RR(1,2)   RR(1,3)    3.085
     RR(2,1)    RR(2,2)   RR(2,3)    1.38
     RR(3,1)    RR(3,2)   RR(3,3)    2.32
     0    0    0    1.0000];
     
R = [T(1,1)    T(1,2)   T(1,3)
     T(2,1)    T(2,2)   T(2,3)
     T(3,1)    T(3,2)   T(3,3)]; 

o = [T(1,4); T(2,4); T(3,4)];

xc = o(1) - L6*R(1,3);
yc = o(2) - L6*R(2,3);
zc = o(3) - L6*R(3,3);

% find thetha1
thetha1 = atan2d(yc,xc)

% find thetha3
a = sqrt(L4^2+L5^2);
r = sqrt(xc^2+yc^2)-L1;
s = zc - L2;
b = sqrt(r^2+s^2);
D = (L3^2+a^2-b^2)/(2*L3*a);
fi = acosd(D);
beta = 180 - fi;
alpha = atan2d(L4 ,L5);
thetha3 = beta + alpha + 90

% find thetha2
fi2 = atan2d(s,r);
D1 = (L3^2 + b^2 - a^2)/(2*L3*b);
fi1 = acosd(D1);
thetha2 = fi2 - fi1

% find thetha4, thetha5, thetha6
alpha1 = 90; alpha2 = 0; alpha3 = 90;
r1 = L1; r2 = L3; r3 = -L4;
d1 = L2; d2 = 0; d3 = 0; 

T1 = [cosd(thetha1) -sind(thetha1)*cosd(alpha1) sind(thetha1)*sind(alpha1) r1*cosd(thetha1);
      sind(thetha1) cosd(thetha1)*cosd(alpha1) -cosd(thetha1)*sind(alpha1) r1*sind(thetha1);
      0 sind(alpha1) cosd(alpha1) d1;
      0 0 0 1];
T2 = [cosd(thetha2) -sind(thetha2)*cosd(alpha2) sind(thetha2)*sind(alpha2) r2*cosd(thetha2);
      sind(thetha2) cosd(thetha2)*cosd(alpha2) -cosd(thetha2)*sind(alpha2) r2*sind(thetha2);
      0 sind(alpha2) cosd(alpha2) d2;
      0 0 0 1];
T3 = [cosd(thetha3) -sind(thetha3)*cosd(alpha3) sind(thetha3)*sind(alpha3) r3*cosd(thetha3);
      sind(thetha3) cosd(thetha3)*cosd(alpha3) -cosd(thetha3)*sind(alpha3) r3*sind(thetha3);
      0 sind(alpha3) cosd(alpha3) d3;
      0 0 0 1];
 
T03 = T1*T2*T3;
R03 = [T03(1,1)    T03(1,2)   T03(1,3)
       T03(2,1)    T03(2,2)   T03(2,3)
       T03(3,1)    T03(3,2)   T03(3,3)];

R03T = R03.'; 
R36 = R03T*R;

% find thetha5
thetha5 = acosd(-R36(3,3))-180

% find thetha4
thetha4 = atan2d(-R36(2,3),-R36(1,3))

% find thetha6
thetha6 = atan2d(-R36(3,2),R36(3,1))
% thetha1 = 30; thetha2 = 15; thetha3 = 120; thetha4 = 5; thetha5 = -60; thetha6 = 10;




% KUKA Robot arm RTB

% L1 = 0.729; L2 = 1.546; L3 = 1.949; L4 = 0.084; L5 = 1.505; L6 = 0.968;
% thetha1 = 0; thetha2 = 90; thetha3 = 0; thetha4 = 0; thetha5 = 0; thetha6 = 0;
% thetha1 = 29.46; thetha2 = -0.48; thetha3 = 147.7; thetha4 = 0; thetha5 = 32.76; thetha6 = 150.54;
alpha1 = 90; alpha2 = 0; alpha3 = 90; alpha4 = 270; alpha5 = 90; alpha6 = 0;
r1 = L1; r2 = L3; r3 = -L4; r4 = 0; r5 = 0; r6 = 0; 
d1 = L2; d2 = 0; d3 = 0; d4 = L5; d5 = 0; d6 = L6;

L(1) = Revolute('d',d1,'a',r1,'alpha',alpha1*pi/180);
L(2) = Revolute('d',d2,'a',r2,'alpha',alpha2*pi/180);
L(3) = Revolute('d',d3,'a',r3,'alpha',alpha3*pi/180);
L(4) = Revolute('d',d4,'a',r4,'alpha',alpha4*pi/180);
L(5) = Revolute('d',d5,'a',r5,'alpha',alpha5*pi/180);
L(6) = Revolute('d',d6,'a',r6,'alpha',alpha6*pi/180);
robot = SerialLink(L);
joints = [thetha1*pi/180,thetha2*pi/180,thetha3*pi/180,thetha4*pi/180,thetha5*pi/180,thetha6*pi/180];
robot.plot(joints);
robot.teach(joints);
%% R36
clc
clear
L4 = 0.084; L5 = 1.505; L6 = 0.968;
alpha4 = 270; alpha5 = 90; alpha6 = 0;
r4 = 0; r5 = 0; r6 = 0; 
d4 = L5; d5 = 0; d6 = L6;

syms R4 R5 R6 R36 thetha4 thetha5 thetha6

R4 = [cos(thetha4) -sin(thetha4)*cosd(alpha4) sin(thetha4)*sind(alpha4);
      sin(thetha4) cos(thetha4)*cosd(alpha4) -cos(thetha4)*sind(alpha4);
      0 sind(alpha4) cosd(alpha4)];
R5 = [cos(thetha5) -sin(thetha5)*cosd(alpha5) sin(thetha5)*sind(alpha5);
      sin(thetha5) cos(thetha5)*cosd(alpha5) -cos(thetha5)*sind(alpha5);
      0 sind(alpha5) cosd(alpha5)];
R6 = [cos(thetha6) -sin(thetha6)*cosd(alpha6) sin(thetha6)*sind(alpha6);
      sin(thetha6) cos(thetha6)*cosd(alpha6) -cos(thetha6)*sind(alpha6);
      0 sind(alpha6) cosd(alpha6)];

R36 = R4*R5*R6

%% KUKA Robot arm RTB

L1 = 0.729; L2 = 1.546; L3 = 1.949; L4 = 0.084; L5 = 1.505; L6 = 0.968;
% thetha1 = 0; thetha2 = 90; thetha3 = 0; thetha4 = 0; thetha5 = 0; thetha6 = 0;
thetha1 = 30; thetha2 = 15; thetha3 = 120; thetha4 = 5; thetha5 = -60; thetha6 = 10;
alpha1 = 90; alpha2 = 0; alpha3 = 90; alpha4 = 270; alpha5 = 90; alpha6 = 0;
r1 = L1; r2 = L3; r3 = -L4; r4 = 0; r5 = 0; r6 = 0; 
d1 = L2; d2 = 0; d3 = 0; d4 = L5; d5 = 0; d6 = L6;

L(1) = Revolute('d',d1,'a',r1,'alpha',alpha1*pi/180);
L(2) = Revolute('d',d2,'a',r2,'alpha',alpha2*pi/180);
L(3) = Revolute('d',d3,'a',r3,'alpha',alpha3*pi/180);
L(4) = Revolute('d',d4,'a',r4,'alpha',alpha4*pi/180);
L(5) = Revolute('d',d5,'a',r5,'alpha',alpha5*pi/180);
L(6) = Revolute('d',d6,'a',r6,'alpha',alpha6*pi/180);
robot = SerialLink(L);
joints = [thetha1*pi/180,thetha2*pi/180,thetha3*pi/180,thetha4*pi/180,thetha5*pi/180,thetha6*pi/180];
robot.plot(joints);
robot.teach(joints);

%%
clc
clear

L1 = 0.729; L2 = 1.546; L3 = 1.949; L4 = 0.084; L5 = 1.505; L6 = 0.968;
% thetha1 = 0; thetha2 = 90; thetha3 = 0; thetha4 = 0; thetha5 = 0; thetha6 = 0;
thetha1 = 30; thetha2 = 15; thetha3 = 120; thetha4 = 5; thetha5 = -60; thetha6 = 10;
alpha1 = 90; alpha2 = 0; alpha3 = 90; alpha4 = 270; alpha5 = 90; alpha6 = 0;
r1 = L1; r2 = L3; r3 = -L4; r4 = 0; r5 = 0; r6 = 0; 
d1 = L2; d2 = 0; d3 = 0; d4 = L5; d5 = 0; d6 = L6;

T1 = [cosd(thetha1) -sind(thetha1)*cosd(alpha1) sind(thetha1)*sind(alpha1) r1*cosd(thetha1);
      sind(thetha1) cosd(thetha1)*cosd(alpha1) -cosd(thetha1)*sind(alpha1) r1*sind(thetha1);
      0 sind(alpha1) cosd(alpha1) d1;
      0 0 0 1];
T2 = [cosd(thetha2) -sind(thetha2)*cosd(alpha2) sind(thetha2)*sind(alpha2) r2*cosd(thetha2);
      sind(thetha2) cosd(thetha2)*cosd(alpha2) -cosd(thetha2)*sind(alpha2) r2*sind(thetha2);
      0 sind(alpha2) cosd(alpha2) d2;
      0 0 0 1];
T3 = [cosd(thetha3) -sind(thetha3)*cosd(alpha3) sind(thetha3)*sind(alpha3) r3*cosd(thetha3);
      sind(thetha3) cosd(thetha3)*cosd(alpha3) -cosd(thetha3)*sind(alpha3) r3*sind(thetha3);
      0 sind(alpha3) cosd(alpha3) d3;
      0 0 0 1];
T4 = [cosd(thetha4) -sind(thetha4)*cosd(alpha4) sind(thetha4)*sind(alpha4) r4*cosd(thetha4);
      sind(thetha4) cosd(thetha4)*cosd(alpha4) -cosd(thetha4)*sind(alpha4) r4*sind(thetha4);
      0 sind(alpha4) cosd(alpha4) d4;
      0 0 0 1];
T5 = [cosd(thetha5) -sind(thetha5)*cosd(alpha5) sind(thetha5)*sind(alpha5) r5*cosd(thetha5);
      sind(thetha5) cosd(thetha5)*cosd(alpha5) -cosd(thetha5)*sind(alpha5) r5*sind(thetha5);
      0 sind(alpha5) cosd(alpha5) d5;
      0 0 0 1];
T6 = [cosd(thetha6) -sind(thetha6)*cosd(alpha6) sind(thetha6)*sind(alpha6) r6*cosd(thetha6);
      sind(thetha6) cosd(thetha6)*cosd(alpha6) -cosd(thetha6)*sind(alpha6) r6*sind(thetha6);
      0 sind(alpha6) cosd(alpha6) d6;
      0 0 0 1];

T1*T2*T3*T4*T5*T6