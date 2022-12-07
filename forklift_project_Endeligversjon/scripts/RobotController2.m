close all
import ETS3.*

L1=0.50;
L2=1.00;
L3=1.00;
L4=1.00;
L5=0.50;
L6=0.50;

%robot_normal = Tz(L1) * Rz('q1') * Tz(L2) * Ry('q2') * Tz(L3) * Ry('q3') * Tz(L4) * Ry('q4') * Tz(L5);
%robot_normal.plot([0,0,0,0])
%
%DH paramters to make kinematic model
j1 = Revolute('d', 0, 'a', L1, 'alpha', pi/2, 'offset', 0);
j2 = Revolute('d', 0, 'a', L2, 'alpha', 0, 'offset', 0);
j3 = Revolute('d', 0, 'a', L3, 'alpha', 0, 'offset', 0);
j4 = Revolute('d', 0, 'a', L4, 'alpha', 0, 'offset', 0);
j5 = Revolute('d', 0, 'a', L5, 'alpha', -pi/2, 'offset', 0);
j6 = Revolute('d', 0, 'a', L6, 'alpha', 0, 'offset', 0);


robot = SerialLink([j1 j2 j3 j4 j5 j6],'name', 'my robot');

robot.plot([0,0,0,0,0,0])
