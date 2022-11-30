clc;
clear all;

L(1) = Link([0,0,0.50,pi/2]);
L(2) = Link([0,0,1.00,0]);
L(3) = Link([0,0,1.00,0]);
L(4) = Link([0,0,1.00,0]);
L(5) = Link([0,0,0.5,-pi/2]);
L(6) = Link([0,0,0.5,0]);

my_robot = SerialLink(L)

L2.qlim = [-100, 100];
L3.qlim = [-100, 100];
L4.qlim = [-100, 100];
L5.qlim = [-100, 100];



%Forward kinematics toolbox
T0 = my_robot.fkine([0, 0, 0, 0, 0, 0])



%inverse kinematics
palle = transl(3, 0, -1) * trotx(0)

palleLoft = transl(3, 0, -0.9) * trotx(0)

taVekk = transl(2.5, 1, 0) *trotx(0)

taPaPlass = transl(3, 2, 1.1) * trotx(0)

taPaPlassSettNed = transl(3, 2, 1) * trotx(0)

palle = transl(3.5, 0, -1) * rpy2tr(0,0,0,'deg')

palleLoft = transl(3.5, 0, -0.9) * rpy2tr(0,0,0, 'deg')

taVekk = transl(2, 0, 0) *rpy2tr(0,0,0, 'deg')

taPaPlass = transl(3, 2, 1.1) * rpy2tr(0,0,0, 'deg')

taPaPlassSettNed = transl(3, 2, 1) * rpy2tr(0,0,0, 'deg')

hold on
trplot(palle)

hold on 
trplot(taPaPlassSettNed)

v1 = ctraj(palle, palleLoft, 50);

v2 = ctraj(palleLoft, taVekk, 50);

v3 = ctraj(taVekk, taPaPlass, 50);

v4 = ctraj(taPaPlass, taPaPlassSettNed, 50);

rV1 = my_robot.ikcon(v1);

rV2 = my_robot.ikcon(v2);

rV3 = my_robot.ikcon(v3);

rV4 = my_robot.ikcon(v4);

my_robot.plot(rV1)

hold on

my_robot.plot(rV2)

hold on

my_robot.plot(rV3)

hold on

my_robot.plot(rV4)

figure
qplot(v2)

figure 
qplot(v3)



%Differential kinematics
J0 = my_robot.jacob0([0 0 1 1 0 0])         %Joint velocities world frame coodrinates

JE = my_robot.jacobe([0 0 1 1 0 0])         %Joint velocities end effector coordinate frame

determinant = det(J0)                       %If zero, at singularity

rank = rank(J0)

Jinverse = J0^-1                            %Cartesian velocities

my_robot.teach([0,0,1,1,0,0],'callback', @(r,q) r.vellipse (q))
