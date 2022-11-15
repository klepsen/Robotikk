clc;
clear all;

L(1) = Link('revolute','d',0,'a',50,'alpha',pi/2);
L(2) = Link('revolute','d',0,'a',100,'alpha',0);
L(3) = Link('revolute','d',0,'a',100,'alpha',0);
L(4) = Link('revolute','d',0,'a',100,'alpha',0);
L(5) = Link('revolute','d',0,'a',50,'alpha',pi/2);
L(6) = Link('revolute','d',0,'a',50,'alpha',0);

% syms q1 q2 q3 q4 q5 q6
% 
% L(1) = Link([0,0,30,pi/2]);
% L(2) = Link([0,0,100,0]);
% L(3) = Link([0,0,100,0]);
% L(4) = Link([0,0,100,0]);
% L(5) = Link([0,0,30,-pi/2]);
% L(6) = Link([0,0,30,0]);


my_robot = SerialLink(L)

% my_robot.teach

% %Forward kinematics without toolbox
% a1 = 30;
% a2 = 100;
% a3 = 100;
% a4 = 100;
% 
% d = 0;
% 
% q1 = 0;
% q2 = 0;
% q3 = 0;
% q4 = 0;
% 
% alpha1 = pi/2;
% alpha = 0;
% 
% T4til3 = [cos(q4), -sin(q4), 0, a4*cos(q4),
%     sin(q4), cos(q4), 0, a4*sin(q4),
%     0, sin(alpha), cos(alpha), 0
%     0, 0, 0, 1]
% 
% T3til2 = [cos(q3), -sin(q3), 0, a3*cos(q3),
%     sin(q3), cos(q3), 0, a3*sin(q3),
%     0, sin(alpha), cos(alpha), 0
%     0, 0, 0, 1]
% 
% T2til1 = [cos(q2), -sin(q2), 0, a2*cos(q2),
%     sin(q2), cos(q2), 0, a2*sin(q2),
%     0, sin(alpha), cos(alpha), 0
%     0, 0, 0, 1]
% 
% T1til0 = [cos(q1), -sin(q1), 0, a1*cos(q1),
%     sin(q1), cos(q1), 0, a1*sin(q1),
%     0, sin(alpha1), cos(alpha1), 0
%     0, 0, 0, 1]
% 
% T0til4 = T1til0 * T2til1 * T3til2 * T4til3


%Forward kinematics toolbox
% T = my_robot.fkine([0, 0, 0, 0, 0, 0]);
% 
% sT = simplify(T)
% 
% vsT = vpa(sT,4)

% J0 = my_robot.jacob0([0 0 0 0 0 0])

% my_robot.teach

% my_robot.plot([0,45,0,0,0,0], 'deg')

my_robot.plot([0 0 0 0 0 0])

palle = transl(350, 0, -100) * rpy2tr(0, 0, 0, 'deg')

taVekk = transl(200, 0, -10) * rpy2tr(0, 0, 0, 'deg')

taPaPlass = transl(350, 0, 100) * rpy2tr(0, 0, 0, 'deg')

hold on
trplot(palle)

startT = my_robot.ikine(palle)

taVekkT = my_robot.ikine(taVekk)

taPaPlassT = my_robot.ikine(taPaPlass)

v2 = jtraj(startT, taVekkT, 50);

v3 = jtraj(taVekkT, taPaPlassT, 50);

% hold on
% my_robot.plot(v1)

hold on
my_robot.plot(v2)

hold on 
my_robot.plot(v3)

% T = transl(100, 0, -200)
% 
% I = my_robot.ikine(T)
% 
% t = transl(100, 0, 200)
% 
% i = my_robot.ikine(t)
% 
% tg = jtraj(I,i,50);
% 
% my_robot.plot(tg)

% figure
% my_robot.teach

% ps = [0 0.5 0.5 0 -2 0]
% 
% psl = [0 0 0 0 0 0]
% 
% tg = jtraj(ps,psl,50);
% 
% my_robot.plot(tg)


