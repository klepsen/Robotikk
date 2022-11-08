clc;
clear all;
close all

syms q1 q2 q3 q4 a1 a2 a3 a4 k

%Forward kinematics
L(1) = Link('revolute','d',0,'a',115.5,'alpha',pi/2);
L(2) = Link('revolute','d',0,'a',115.5,'alpha',0);
L(3) = Link('revolute','d',0,'a',60,'alpha',0);
L(4) = Link('revolute','d',0,'a',40,'alpha',0);

% L(1) = Link([0,0,115.5,pi/2]);
% L(2) = Link([0,0,115.5,0]);
% L(3) = Link([0,0,60,0]);
% L(4) = Link([0,0,40,0]);

my_robot = SerialLink(L)

%my_robot.teach
 
T = my_robot.fkine([2,5,6,25])

inverse = my_robot.ikine(T,'mask',[1 1 1 0 1 0])

Tst = transl(0, 0, 0) * rpy2tr(0, 0 , 0, 'deg')

Ts = transl(0.6, 0, 0) * rpy2tr(0, 0 , 45, 'deg')

Qst = my_robot.ikine(Tst, 'mask', [1 1 1 0 0 1])
Qs = my_robot.ikine(Ts, 'mask', [1 1 1 0 0 1])

% tg = jtraj(;

my_robot.plot(Qs)

% T = my_robot.fkine([q1 q2 q3 q4])

% my_robot.plot([0,18,18,18],'deg')
% 
% axis ([-400 400, -400 400, -400 400])
% 
% 
% T1 = SE3(0.4, 0.2, 0) * SE3.Rz(pi)
% T2 = SE3(0.4, -0.2, 0) * SE3.Rz(pi/2);
% 
% q1 = my_robot.ikine(T1, 'mask', [1 1 1 0 0 1])
% q2 = my_robot.ikine(T2, 'mask', [1 1 1 0 0 1])
% 
% t = [0:0.05:2]'; 
% q = mtraj(@tpoly, q1, q2, t); 
% % figure
% % plot(q)
% 
% Ts = ctraj(T1, T2, length(t));
% % figure
% % plot(t, Ts.transl); 
% qc = my_robot.ikine(T2, 'mask', [1 1 1 0 0 1]);


% T4til3 = [cos(q4), -sin(q4), 0, a4*cos(q4),
%     sin(q4), cos(q4), 0, a4*sin(q4),
%     0, 0, 1, 0
%     0, 0, 0, 1]
% 
% T3til2 = [cos(q3), -sin(q3), 0, a3*cos(q3),
%     sin(q3), cos(q3), 0, a3*sin(q3),
%     0, 0, 1, 0
%     0, 0, 0, 1]
% 
% T2til1 = [cos(q2), -sin(q2), 0, a2*cos(q2),
%     sin(q2), cos(q2), 0, a2*sin(q2),
%     0, 0, 1, 0
%     0, 0, 0, 1]
% 
% T1til0 = [cos(q1), -sin(q1), 0, a1*cos(q1),
%     sin(q1), cos(q1), 0, a1*sin(q1),
%     0, 0, 1, 0
%     0, 0, 0, 1]
% 
% T0til4 = T1til0 * T2til1 * T3til2 * T4til3
% 
% simplify(T0til4)


